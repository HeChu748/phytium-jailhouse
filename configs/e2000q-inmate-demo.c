/*
 * Jailhouse, a Linux-based partitioning hypervisor
 *
 * Configuration for Phytium E2000Q
 * Copyright (c) 2022-2023 Phytium Technology Co., Ltd
 *
 * Authors:
 *  Shaojun Yang <yangshaojun@phytium.com.cn>
 *  huyuming <huyuming1672@phytium.com.cn>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.
 * See the COPYING file in the top-level directory.
 */

 #include <jailhouse/types.h>
 #include <jailhouse/cell-config.h>
 
 struct {
	 struct jailhouse_cell_desc cell;
	 __u64 cpus[1];
	 struct jailhouse_memory mem_regions[10];
	 struct jailhouse_irqchip irqchips[1];
	 struct jailhouse_pci_device pci_devices[1];
 } __attribute__((packed)) config = {
	 .cell = {
		 .signature = JAILHOUSE_CELL_DESC_SIGNATURE,
		 .revision = JAILHOUSE_CONFIG_REVISION,
		 .name = "inmate-demo",
		 .flags = JAILHOUSE_CELL_PASSIVE_COMMREG | JAILHOUSE_CELL_VIRTUAL_CONSOLE_PERMITTED,
 
		 .cpu_set_size = sizeof(config.cpus),
		 .num_memory_regions = ARRAY_SIZE(config.mem_regions),
		 .num_irqchips = ARRAY_SIZE(config.irqchips),
		 .num_pci_devices = ARRAY_SIZE(config.pci_devices),
 
		 .vpci_irq_base = 71-32,
 
		 .console = {
			 .address = 0x2800d000,
			 .type    = JAILHOUSE_CON_TYPE_PL011,
			 .flags   = JAILHOUSE_CON_ACCESS_MMIO | JAILHOUSE_CON_REGDIST_4,
		 },
	 },
 
	 .cpus = {
		 0x2,
	 },
 
	 .mem_regions = {
		 /* IVSHMEM shared memory regions (demo) */
		 {
			 .phys_start = 0xb1000000,
			 .virt_start = 0xb1000000,
			 .size		= 0x1000,
			 .flags		= JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | JAILHOUSE_MEM_ROOTSHARED,
		 },
		 {
			 .phys_start = 0xb1001000,
			 .virt_start = 0xb1001000,
			 .size		= 0x9000,
			 .flags		= JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | JAILHOUSE_MEM_ROOTSHARED,
		 },
		 {
			 .phys_start = 0xb100a000,
			 .virt_start = 0xb100a000,
			 .size		= 0x2000,
			 .flags		= JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | JAILHOUSE_MEM_ROOTSHARED,
		 },
		 {
			 .phys_start = 0xb100c000,
			 .virt_start = 0xb100c000,
			 .size		= 0x2000,
			 .flags		= JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | JAILHOUSE_MEM_ROOTSHARED,
		 },
		 {
			 .phys_start = 0xb100e000,
			 .virt_start = 0xb100e000,
			 .size		= 0x2000,
			 .flags		= JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | JAILHOUSE_MEM_ROOTSHARED,
		 },

 
		 /* UART 1 */ {
			 .phys_start = 0x2800d000,
			 .virt_start = 0x2800d000,
			 .size		= 0x1000,
			 .flags		= JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | JAILHOUSE_MEM_IO | JAILHOUSE_MEM_ROOTSHARED,
		 },
 
		/* GPIO2 */ {
			.phys_start = 0x28036000,
			.virt_start = 0x28036000,
			.size		= 0x1000,
			.flags		= JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | JAILHOUSE_MEM_IO | JAILHOUSE_MEM_ROOTSHARED,
		 },

		/* GPIO3 */ {
			.phys_start = 0x28037000,
			.virt_start = 0x28037000,
			.size		= 0x1000,
			.flags		= JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | JAILHOUSE_MEM_IO | JAILHOUSE_MEM_ROOTSHARED,
		 },
 
		 /* RAM */ {
			 .phys_start = 0xc2000000,
			 .virt_start = 0x0,
			 .size		= 0x01000000,
			 .flags		= JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | JAILHOUSE_MEM_EXECUTE | JAILHOUSE_MEM_LOADABLE,
		 },
 
		 /* communication region */ {
			 .virt_start = 0x80000000,
			 .size		= 0x00001000,
			 .flags		= JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | JAILHOUSE_MEM_COMM_REGION,
		 },
	 },
 
	 .irqchips = {
		 /* GIC */
		 {
			 .address = 0x30800000,
			 .pin_base = 32,
			 .pin_bitmap = {
				 0,
				 0x1 << (71 - 64),
				 0,
				 0
			 },
		 },
	 },
 
	 .pci_devices = {
		 {
			 .type		= JAILHOUSE_PCI_TYPE_IVSHMEM,
			 .domain		= 0,
			 .bdf		= 0 << 3,
			 .bar_mask	= JAILHOUSE_IVSHMEM_BAR_MASK_INTX,
			 .shmem_regions_start = 0,
			 .shmem_dev_id = 2,
			 .shmem_peers = 3,
			 .shmem_protocol = JAILHOUSE_SHMEM_PROTO_UNDEFINED,
		 },
	 },
 
 };
 
 