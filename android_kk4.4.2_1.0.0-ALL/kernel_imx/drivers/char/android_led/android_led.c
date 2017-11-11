#include <linux/module.h>               /* For module specific items */  
#include <linux/moduleparam.h>          /* For new moduleparam's */  
#include <linux/types.h>                /* For standard types (like size_t) */  
#include <linux/errno.h>                /* For the -ENODEV/... values */  
#include <linux/kernel.h>               /* For printk/panic/... */  
#include <linux/fs.h>                   /* For file operations */
#include <linux/ioport.h>               /* For io-port access */  
#include <linux/platform_device.h>      /* For platform_driver framework */  
#include <linux/init.h>                 /* For __init/__exit/... */  
#include <linux/uaccess.h>              /* For copy_to_user/put_user/... */  
#include <linux/io.h>                   /* For inb/outb/... */  
#include <linux/gpio.h>  
#include <linux/device.h>  
#include <linux/cdev.h>  
#include <linux/slab.h>               /*kamlloc */  
//#include <asm-generic/ioctl.h>  
   
  
#define LOW                     (0)
#define HIGH                    (1)

#define IMX_GPIO_NR(bank, nr)	(((bank) - 1) * 32 + (nr))
#define SABRESD_KYB_LEDEN       IMX_GPIO_NR(6, 11) 
#define KYB_LED_CTR        (9) 

  
#define CMD_FLAG 'i' 
#define LED_ON _IOR(CMD_FLAG,0x00000000,__u32) 
#define LED_OFF _IOR(CMD_FLAG,0x00000001,__u32) 

static int major =0; 
static struct class *led_class; 
struct cdev_led { 
struct cdev cdev; 
};  
struct cdev_led *led_dev; 

static int led_ioctl(struct file* filp,unsigned int cmd,unsigned long argv) 
{ 
	printk(KERN_INFO "entry kernel.... \n"); 

	switch(cmd) 
	{ 
	case LED_ON: 
		{ 
			gpio_direction_output(SABRESD_KYB_LEDEN, 0);
			gpio_set_value(SABRESD_KYB_LEDEN,1);
			printk("keyboard_backlight control pin turn on!\r\n");
			break; 
		} 
	case LED_OFF: 
		{ 
			gpio_direction_output(SABRESD_KYB_LEDEN, 0);
			gpio_set_value(SABRESD_KYB_LEDEN,0);
			printk("keyboard_backlight control pin turn on!\r\n");
			break; 
		} 
	default: 
	return -EINVAL; 
	} 
		return 0; 
} 


//open 
static int led_open(struct inode* i_node,struct file* filp) 
{ 
printk(KERN_INFO "open init.... \n"); 
int err; 
err = gpio_request(SABRESD_KYB_LEDEN,"led1");  
if(err<0) 
{ 
printk(KERN_INFO "gpio request faile \n"); 
return err; 
} 
gpio_direction_output(SABRESD_KYB_LEDEN,1); 

return 0; 
} 

//close 
static void led_close(struct inode* i_node,struct file* filp) 
{ 
printk(KERN_INFO "close init \n"); 
gpio_free(SABRESD_KYB_LEDEN); 
return ; 
} 

/* file operations */ 
struct file_operations led_fops={ 
.owner = THIS_MODULE, 
.open = led_open, 
.unlocked_ioctl = led_ioctl,
.release= led_close, 
}; 

static int __init led_init(void) 
{ 
    printk(KERN_INFO "init .... \n"); 
    dev_t dev_no; 
    int result,err; 
    err = alloc_chrdev_region(&dev_no,0,1,"my_led"); 
    if(err<0) 
    { 
        printk(KERN_INFO "ERROR\n"); 
        return err; 
    } 
    major = MAJOR(dev_no); 
    led_dev = kmalloc(sizeof(struct cdev_led),GFP_KERNEL); 
    if(!led_dev) 
    { 
        result = -ENOMEM; 
        goto fail_malloc; 
    } 
    memset(led_dev,0,sizeof(led_dev)); 

    cdev_init(&led_dev->cdev,&led_fops); 
    led_dev->cdev.owner = THIS_MODULE; 
    result = cdev_add(&led_dev->cdev,dev_no,1); 
    if(result <0) 
    { 
        printk(KERN_INFO "error\n"); 
        goto fail_add; 
    } 
    led_class = class_create(THIS_MODULE,"myled"); 
    device_create(led_class,NULL,MKDEV(major,0),NULL,"myled"); 
    return 0; 
        fail_add: 
        kfree(led_dev); 
        fail_malloc: 
        unregister_chrdev_region(dev_no,1); 
    return result; 

} 

static void __exit led_exit(void) 
{ 
dev_t dev_no=MKDEV(major,0); 

unregister_chrdev_region(dev_no,1); 
cdev_del(&led_dev->cdev); 
kfree(led_dev); 
device_destroy(led_class,dev_no); 
class_destroy(led_class); 
printk(KERN_INFO "exit........ \n"); 
} 
module_init(led_init); 
module_exit(led_exit); 
MODULE_AUTHOR("koliy "); 
MODULE_DESCRIPTION("ARM test led"); 
MODULE_LICENSE("GPL"); 
