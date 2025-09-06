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

#define GPIO3_BASE      0x28037000  
#define GPIO_SWPORTA_DR     0x00
#define GPIO_SWPORTA_DDR    0x04
#define GPIO_EXT_PORTA      0x50 

#define REG32(addr)         (*(volatile unsigned int *)(addr))
#define GPIO_REG(offset)    REG32(GPIO3_BASE + (offset))

#define LED_PIN     1


void gpio_set_output(int pin);
void gpio_set_input(int pin);
void gpio_set_high(int pin);
void gpio_set_low(int pin);
int gpio_read(int pin);
void delay(volatile unsigned int count);


void gpio_set_output(int pin)
{
    unsigned int val;
    val = GPIO_REG(GPIO_SWPORTA_DDR);
    val |= (1 << pin);        
    GPIO_REG(GPIO_SWPORTA_DDR) = val;
}


void gpio_set_input(int pin)
{
    unsigned int val;
    val = GPIO_REG(GPIO_SWPORTA_DDR);
    val &= ~(1 << pin);                
    GPIO_REG(GPIO_SWPORTA_DDR) = val;
}


void gpio_set_high(int pin)
{
    unsigned int val;
    val = GPIO_REG(GPIO_SWPORTA_DR);
    val |= (1 << pin);
    GPIO_REG(GPIO_SWPORTA_DR) = val;
}


void gpio_set_low(int pin)
{
    unsigned int val;
    val = GPIO_REG(GPIO_SWPORTA_DR);
    val &= ~(1 << pin);
    GPIO_REG(GPIO_SWPORTA_DR) = val;
}


int gpio_read(int pin)
{
    unsigned int val;
    val = GPIO_REG(GPIO_EXT_PORTA);
    return (val & (1 << pin)) ? 1 : 0;
}


void delay(volatile unsigned int count)
{
    while(count--) ;
}

void inmate_main(void)
{
    gpio_set_output(LED_PIN);

    gpio_set_high(LED_PIN);
    delay(1000000);

    gpio_set_low(LED_PIN);
    delay(1000000);

    gpio_set_input(LED_PIN);
    int val = gpio_read(LED_PIN);
    printk("val = %d\n", val);

    while (1);
}
