/*
 * Jailhouse, a Linux-based partitioning hypervisor
 *
 * Copyright (c) Siemens AG, 2014-2020
 *
 * Authors:
 *  Henning Schild <henning.schild@siemens.com>
 *  Jan Kiszka <jan.kiszka@siemens.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include <inmate.h>

#define I2C3_BASE            0x28024000
#define IC_CON               (*(volatile unsigned int *)(I2C3_BASE + 0x00))
#define IC_TAR               (*(volatile unsigned int *)(I2C3_BASE + 0x04))
#define IC_DATA_CMD          (*(volatile unsigned int *)(I2C3_BASE + 0x10))
#define IC_ENABLE            (*(volatile unsigned int *)(I2C3_BASE + 0x6C))
#define IC_STATUS            (*(volatile unsigned int *)(I2C3_BASE + 0x70))
#define creg_mio_func_sel    (*(volatile unsigned int *)(I2C3_BASE + 0x1000))
#define IC_STATUS_TFNF       (1 << 1)  

 
#define VENDORID                        0x110a
#define DEVICEID                        0x4106
#define BAR_BASE                        0xff000000
#define IVSHMEM_CFG_STATE_TAB_SZ        0x04
#define IVSHMEM_CFG_RW_SECTION_SZ       0x08
#define IVSHMEM_CFG_OUT_SECTION_SZ      0x10
#define IVSHMEM_CFG_ADDRESS             0x18
#define JAILHOUSE_SHMEM_PROTO_UNDEFINED 0x0000

#if defined(__x86_64__)
# define DEFAULT_IRQ_BASE               32
#elif defined(__arm__) || defined(__aarch64__)
# define DEFAULT_IRQ_BASE               (comm_region->vpci_irq_base + 32)
#else
# error Not implemented!
#endif

#define MAX_VECTORS     4


struct ivshm_regs {
    u32 id;
    u32 max_peers;
    u32 int_control;
    u32 doorbell;
    u32 state;
};

struct ivshmem_dev_data {
    u16 bdf;
    struct ivshm_regs *registers;
    u32 *state_table;
    u32 state_table_sz;
    u32 *rw_section;
    u64 rw_section_sz;
    s32 *in_sections;   
    s32 *out_section;   
    u64 out_section_sz;
    u32 *msix_table;
    u32 id;
    int msix_cap;
};

static struct ivshmem_dev_data dev;
static unsigned int irq_base, vectors, target;
static int irq_counter[MAX_VECTORS];


static void delay_cycles(volatile unsigned int count) { while (count--); }

static void i2c_master_init(unsigned char slave_addr) {
    creg_mio_func_sel = 0x00;
    IC_ENABLE = 0;
    IC_CON = 0x63;
    IC_TAR = slave_addr;
    IC_ENABLE = 1;
}


static void i2c_write_byte(unsigned char data, int stop_flag) {
    while ((IC_STATUS & IC_STATUS_TFNF) == 0);
    unsigned int cmd = data & 0xFF;
    if (stop_flag) cmd |= (1 << 9);
    IC_DATA_CMD = cmd;
}


static void i2c_master_write_bytes(unsigned char slave_addr,
                                   unsigned char reg_addr,
                                   unsigned char *data,
                                   int len) {
    i2c_master_init(slave_addr);
    i2c_write_byte(reg_addr, 0);
    for (int i = 0; i < len; i++)
        i2c_write_byte(data[i], (i == len - 1));
}


static void write_convert(float s1, float s2, float s3, float s4, 
    float s5, float s6, u16 time,
    u8 *data, u8 *timeArr) 
{
    u16 pos;

    pos = (u16)(2200.0f * s1 / 180.0f + 900.0f);
    data[0] = (u8)(pos >> 8); 
    data[1] = (u8)(pos & 0xFF); 

    s2 = 180.0f - s2;
    pos = (u16)(2200.0f * s2 / 180.0f + 900.0f);
    data[2] = (u8)(pos >> 8);
    data[3] = (u8)(pos & 0xFF);

    s3 = 180.0f - s3;
    pos = (u16)(2200.0f * s3 / 180.0f + 900.0f);
    data[4] = (u8)(pos >> 8);
    data[5] = (u8)(pos & 0xFF);

    s4 = 180.0f - s4;
    pos = (u16)(2200.0f * s4 / 180.0f + 900.0f);
    data[6] = (u8)(pos >> 8);
    data[7] = (u8)(pos & 0xFF);

    pos = (u16)(3320.0f * s5 / 270.0f + 380.0f);
    data[8] = (u8)(pos >> 8);
    data[9] = (u8)(pos & 0xFF);

    pos = (u16)(2200.0f * s6 / 180.0f + 900.0f);
    data[10] = (u8)(pos >> 8);
    data[11] = (u8)(pos & 0xFF);

    timeArr[0] = (u8)(time >> 8);
    timeArr[1] = (u8)(time & 0xFF);
}


static u64 pci_cfg_read64(u16 bdf, unsigned int addr)
{
    return pci_read_config(bdf, addr, 4) |
           ((u64)pci_read_config(bdf, addr + 4, 4) << 32);
}

static void print_shmem(struct ivshmem_dev_data *d)
{
    printk("state[0] = %d\n", d->state_table[0]);
    printk("state[1] = %d\n", d->state_table[1]);
    printk("state[2] = %d\n", d->state_table[2]);

    printk("rw[0] = %d\n", d->rw_section[0]);
    printk("rw[1] = %d\n", d->rw_section[1]);
    printk("rw[2] = %d\n", d->rw_section[2]);

    printk("in@0x0000 = %d\n", d->in_sections[0/4 + 0]);
    printk("in@0x0004 = %d\n", d->in_sections[0/4 + 1]);
    printk("in@0x0008 = %d\n", d->in_sections[0/4 + 2]);
    printk("in@0x000c = %d\n", d->in_sections[0/4 + 3]);
    printk("in@0x0010 = %d\n", d->in_sections[0/4 + 4]);

    printk("in@0x2000 = %d\n", d->in_sections[0x2000/4]);
    printk("in@0x4000 = %d\n", d->in_sections[0x4000/4]);
}


static void irq_handler(unsigned int irq)
{
	unsigned int n;
	u32 value; 
    u8 data[12];
    u8 timeArr[2];

	if (irq < irq_base || irq >= irq_base + vectors)
		return;

    mmio_write32(&dev.registers->state, 1);

	n = irq - irq_base;
	irq_counter[n]++;
	if (dev.msix_cap > 0)
		value = irq_counter[dev.id];
	else
		value = irq_counter[0];
	dev.rw_section[dev.id] = value;

    write_convert(dev.in_sections[0], dev.in_sections[1], dev.in_sections[2],
        dev.in_sections[3], dev.in_sections[4], dev.in_sections[5],
        dev.in_sections[6], data, timeArr);


    // unsigned char write_addr = 0x1d;
    // unsigned char timeArr_addr = 0x1e; /* 假设 id=3 */
    // i2c_master_write_bytes(slave_addr, write_addr, data, 12);
    // i2c_master_write_bytes(slave_addr, timeArr_addr, timeArr, 2);

    unsigned char slave_addr = 0x15;
    unsigned char write_value[4];
    write_value[2] = timeArr[0];
    write_value[3] = timeArr[1];

    for(int id=1;id<6;id++)
    {
        unsigned char write_addr = 0x10 + id;
        write_value[0] = data[id*2 - 2];
        write_value[1] = data[id*2 - 1];  
        i2c_master_write_bytes(slave_addr, write_addr, write_value, 4);
        delay_cycles(1000000);
    }
    
    mmio_write32(&dev.registers->state, 0);
	printk("\nIVSHMEM1: got interrupt %d (#%d)\n", n, irq_counter[n]);
	print_shmem(&dev);
}

static void init_device(struct ivshmem_dev_data *d)
{
    unsigned long baseaddr, addr, size;
    int vndr_cap, n;
    u32 max_peers;

    vndr_cap = pci_find_cap(d->bdf, PCI_CAP_VENDOR);
    if (vndr_cap < 0) {
        printk("IVSHMEM ERROR: missing vendor capability\n");
        stop();
    }

    d->registers = (struct ivshm_regs *)BAR_BASE;
    pci_write_config(d->bdf, PCI_CFG_BAR, (unsigned long)d->registers, 4);
    printk("IVSHMEM: bar0 is at %p\n", d->registers);

    d->msix_table = (u32 *)(BAR_BASE + PAGE_SIZE);
    pci_write_config(d->bdf, PCI_CFG_BAR + 4, (unsigned long)d->msix_table, 4);
    printk("IVSHMEM: bar1 is at %p\n", d->msix_table);

    pci_write_config(d->bdf, PCI_CFG_COMMAND, (PCI_CMD_MEM | PCI_CMD_MASTER), 2);
    map_range((void *)BAR_BASE, 2 * PAGE_SIZE, MAP_UNCACHED);

    d->id = mmio_read32(&d->registers->id);
    printk("IVSHMEM: ID is %u\n", d->id);

    max_peers = mmio_read32(&d->registers->max_peers);
    printk("IVSHMEM: max. peers is %u\n", max_peers);

    target = d->id < max_peers ? (d->id + 1) : 0;
    target = cmdline_parse_int("target", target);

    d->state_table_sz = pci_read_config(d->bdf, vndr_cap + IVSHMEM_CFG_STATE_TAB_SZ, 4);
    d->rw_section_sz  = pci_cfg_read64(d->bdf, vndr_cap + IVSHMEM_CFG_RW_SECTION_SZ);
    d->out_section_sz = pci_cfg_read64(d->bdf, vndr_cap + IVSHMEM_CFG_OUT_SECTION_SZ);
    baseaddr          = pci_cfg_read64(d->bdf, vndr_cap + IVSHMEM_CFG_ADDRESS);

    addr = baseaddr;
    d->state_table = (u32 *)addr;

    addr += d->state_table_sz;
    d->rw_section = (u32 *)addr;

    addr += d->rw_section_sz;
    d->in_sections = (s32 *)addr;

    addr += d->id * d->out_section_sz;
    d->out_section = (s32 *)addr;    

    printk("IVSHMEM: state table is at %p\n", d->state_table);
    printk("IVSHMEM: R/W section is at %p\n", d->rw_section);
    printk("IVSHMEM: input sections start at %p\n", d->in_sections);
    printk("IVSHMEM: output section is at %p\n", d->out_section);

    size = d->state_table_sz + d->rw_section_sz + max_peers * d->out_section_sz;
    map_range((void *)baseaddr, size, MAP_CACHED);

    d->msix_cap = pci_find_cap(d->bdf, PCI_CAP_MSIX);
    vectors = d->msix_cap > 0 ? MAX_VECTORS : 1;
    for (n = 0; n < (int)vectors; n++) {
        if (d->msix_cap > 0)
            pci_msix_set_vector(d->bdf, irq_base + n, n);
        irq_enable(irq_base + n);
    }
}

static void send_irq(struct ivshmem_dev_data *d)
{
	u32 int_no = d->msix_cap > 0 ? (d->id + 1) : 0;

	disable_irqs();
	target = 0;
	printk("\nIVSHMEM1: sending IRQ %d to peer %d\n", int_no, target);
	enable_irqs();
	mmio_write32(&d->registers->doorbell, int_no | (target << 16));
}

void inmate_main(void)
{
    unsigned int class_rev;
    int bdf;

    irq_base = cmdline_parse_int("irq_base", DEFAULT_IRQ_BASE);
    irq_init(irq_handler);
    pci_init();

    bdf = pci_find_device(VENDORID, DEVICEID, 0);
    if (bdf == -1) {
        printk("IVSHMEM: No PCI devices found .. nothing to do.\n");
        stop();
    }

    printk("IVSHMEM: Found device at %02x:%02x.%x\n",
           bdf >> 8, (bdf >> 3) & 0x1f, bdf & 0x3);

    class_rev = pci_read_config(bdf, 0x8, 4);
    if (class_rev != (PCI_DEV_CLASS_OTHER << 24 |
                      JAILHOUSE_SHMEM_PROTO_UNDEFINED << 8)) {
        printk("IVSHMEM: class/revision %08x, not supported\n", class_rev);
        stop();
    }

    dev.bdf = bdf;
    init_device(&dev);
    printk("IVSHMEM: initialized device\n");

    mmio_write32(&dev.registers->int_control, 1);
    mmio_write32(&dev.registers->state, 0);

    dev.rw_section[dev.id] = 0;
    dev.out_section[0]     = 0;
    print_shmem(&dev);

    enable_irqs();

    while (1)
    {
        delay_cycles(1000000*60);
		send_irq(&dev);
    }
}
