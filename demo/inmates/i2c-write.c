/*
 * Jailhouse, a Linux-based partitioning hypervisor
 *
 * Copyright (c) ARM Limited, 2014
 *
 * Authors:
 *  Jean-Philippe Brucker <jean-philippe.brucker@arm.com>
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


static void i2c_master_init(unsigned char slave_addr)
{
    creg_mio_func_sel = 0x00;
    IC_ENABLE = 0;         
    IC_CON = 0x63;          
    IC_TAR = slave_addr;   
    IC_ENABLE = 1;     
}


static void i2c_write_byte(unsigned char data, int stop_flag)
{
    while ((IC_STATUS & IC_STATUS_TFNF) == 0);

    unsigned int cmd = data & 0xFF;  
    if (stop_flag)
        cmd |= (1 << 9);         

    IC_DATA_CMD = cmd;
}


static void i2c_master_write_bytes(unsigned char slave_addr,
                            unsigned char reg_addr,
                            unsigned char *data,
                            int len)
{
    int i;
    i2c_master_init(slave_addr);
    i2c_write_byte(reg_addr, 0);

    for (i = 0; i < len; i++)
    {
        int stop_flag = (i == len - 1) ? 1 : 0; 
        i2c_write_byte(data[i], stop_flag);
    }
}


void inmate_main(void)
{
    unsigned char values[4] = {0x12, 0x34, 0x01, 0xF4};
    unsigned char slave_addr = 0x15;
    unsigned char reg_addr = 0x10 + 1;

    i2c_master_write_bytes(slave_addr, reg_addr, values, 4);
    while (1);
   
}
