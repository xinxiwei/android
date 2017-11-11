/*
 * character device wrapper for generic gpio layer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 *
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/atomic.h>
#include <linux/init.h>
#include <linux/genhd.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>

static int gpio_state;

#define SABRESD_485_RD	IMX_GPIO_NR(1,30)
static ssize_t gpio_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%u\n", gpio_state);
}

static ssize_t gpio_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        gpio_state = simple_strtoul(buf, NULL, 10);
		gpio_direction_output(SABRESD_485_RD,gpio_state);

        return size;
}

static struct miscdevice io_ctl_dev = {
        .minor         = MISC_DYNAMIC_MINOR,
        .name         = "io_ctl",                   
};

static DEVICE_ATTR(gpio_state, 0644, gpio_state_show, gpio_state_store);

static struct attribute *io_ctl_attributes[] = {
	&dev_attr_gpio_state.attr,
        NULL,
};

static struct attribute_group io_ctl_attr_group = {
        .attrs = io_ctl_attributes,
};

static int __init io_ctl_dev_init(void)
{
	int ret = 0;
	
	ret = gpio_request(SABRESD_485_RD,"gpio1_30");
	if(ret < 0)
		printk("--------request gpio %d fail!\n", 136);
	
  
	ret = misc_register(&io_ctl_dev);
	if(ret){
		printk(KERN_ERR "misc_register failed\n");
		return ret;
	}

	ret = sysfs_create_group(&io_ctl_dev.this_device->kobj, &io_ctl_attr_group);
        if (ret){
		printk(KERN_ERR "creat attr file failed\n");
		misc_deregister(&io_ctl_dev);
		return ret;
	}

	return 0;
}

static void __exit io_ctl_dev_exit(void)
{
	sysfs_remove_group(&io_ctl_dev.this_device->kobj, &io_ctl_attr_group);
	misc_deregister(&io_ctl_dev);
}

module_init (io_ctl_dev_init);
module_exit (io_ctl_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("forlinx");
MODULE_DESCRIPTION("Misc device for 485 ctl");
