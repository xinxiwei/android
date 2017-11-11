#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <asm/gpio.h>
#include <asm/delay.h>
#include <linux/clk.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/slab.h>               /*kamlloc */ 
#include  <asm/uaccess.h> 

//#include <plat/gpio-cfg.h>

#define LOW                     (0)
#define HIGH                    (1)

#define IMX_GPIO_NR(bank, nr)	(((bank) - 1) * 32 + (nr))


#define SABRESD_KYB_LEDEN       IMX_GPIO_NR(6, 11)
#define KB_BL_CTR        (9)


static struct class *lightctr_class;  
struct cdev_imx6q_backlight {  
    struct cdev cdev;  
}; //建议用2.6的注册方法，2.4的已经离我们越来越远了。  
struct cdev_imx6q_backlight *imx6q_dev_backlight; 
static int  major =0; 

static int imx6q_backlight_status[4] = { 0 };  //gpio控制的状态标志位


static int imx6q_backlight_open(struct inode *inode,struct file *file)
{
    printk(KERN_INFO "get int funticon---> imx6q_backlight_open !\n");

	/* gpio_direction_input(SABRESD_KYB_LEDEN); */
    return 0;
}

static void imx6q_backlight_close(struct inode* i_node,struct file* filp)  
{  
    printk(KERN_INFO "close init \n");  
    /* 
	gpio_free(SABRESD_KYB_LEDEN);  */
    return ;  
}  

static int imx6q_ioctl(struct file* filp,unsigned int cmd,unsigned long argv) 
{  
    int err = 0;
    printk("imx6q_ioctl_backlight cmd = %d, argv = %d\r\n",cmd,argv); 

    switch(cmd){
   
      case KB_BL_CTR:
            if(argv>0){
                err = gpio_direction_output(SABRESD_KYB_LEDEN, LOW);
                printk("keyboard_light control pin turn on!\r\n");
            }else{
                err = gpio_direction_output(SABRESD_KYB_LEDEN, HIGH);
                printk("keyboard_light control pin turn off!\r\n");
            }
            break; 
        default:
            printk("cmd is not exist\n");
            return -1;
    }
    return err;
}  


/* file operations */  
struct file_operations fops_backlight={  
    .owner  = THIS_MODULE,  
    .open   = imx6q_backlight_open,  
    .unlocked_ioctl = imx6q_ioctl, // 特别注意从2.6.36以后ioctl已经移除，内核里面用unlocked_ioctl和compat_ioctl. 应用层不变，仍是ioctl调用。  
    .release= imx6q_backlight_close, 
};

static int imx6q_backlightctl_init(void)  
{  
    printk(KERN_INFO "init .... \n");  
    dev_t dev_no;  
    int result,err; 
 
    err = alloc_chrdev_region(&dev_no,0,1,"dev_backlight"); //动态申请设备号  
    if(err<0)  
    {  
        printk(KERN_INFO "ERROR\n");  
        return err;  
    }  
    major = MAJOR(dev_no);  
    imx6q_dev_backlight = kmalloc(sizeof(struct cdev_imx6q_backlight),GFP_KERNEL);  
    if(!imx6q_dev_backlight)  
    {  
        result = -ENOMEM;  
        goto fail_malloc;  
    }  
    memset(imx6q_dev_backlight,0,sizeof(imx6q_dev_backlight));  
      
    /* major = register_chrdev(0, "led", &led_fops);*/

    cdev_init(&imx6q_dev_backlight->cdev,&fops_backlight); // 初始化cdev  
    imx6q_dev_backlight->cdev.owner = THIS_MODULE;  
    result = cdev_add(&imx6q_dev_backlight->cdev,dev_no,1); //加载设备  
    if(result <0)  
    {   printk(KERN_INFO "error\n");  
        goto fail_add;  
    }  


    lightctr_class = class_create(THIS_MODULE,"imx6qbacklightctr");  //在sys/class下创建sysfs文件  
    device_create(lightctr_class,NULL,MKDEV(major,0),NULL,"devbacklightctr"); //动态创建设备文件  /dev/devbacklightctr 以后不用手动创建了  
    return 0;  
fail_add:  
    kfree(imx6q_dev_backlight);  
fail_malloc:  
    unregister_chrdev_region(dev_no,1);  
    return result;    
}  

static void imx6q_dev_backlight_exit(void)  
{  
    dev_t dev_no=MKDEV(major,0);  
  
    unregister_chrdev_region(dev_no,1);  
    cdev_del(&imx6q_dev_backlight->cdev);  
    kfree(imx6q_dev_backlight);  
    device_destroy(lightctr_class,dev_no);  
    class_destroy(lightctr_class);  
    printk(KERN_INFO "exit........ \n");  
} 

static void imx6q_backlight_probe(void)
{
    int err;
      
    printk("\nget into function -->imx6q_backlight_probe! \n");    
  
     err = gpio_request(SABRESD_KYB_LEDEN,"KYB_LED_CTR");   
    if(err<0)  
    {  
        printk(KERN_INFO "KYB_LED_CTR gpio request faile \n");  
        return ;  
    }
	gpio_direction_input(SABRESD_KYB_LEDEN);
    imx6q_backlightctl_init();  
}



static  void  imx6q_backlight_remove(void)
{
   	gpio_free(SABRESD_KYB_LEDEN);
    imx6q_dev_backlight_exit();
}

 static ssize_t  imx6q_backlight_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    if(!strcmp(attr->attr.name, "keyboardbl_pwn"))
     {
         if(imx6q_backlight_status[3] != 0)
             return  strlcpy(buf, "1\n", 3);
         else
             return  strlcpy(buf, "0\n", 3);
     }
	
     return   strlcpy(buf, "\n", 3);
}

static ssize_t imx6q_backlight_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned long on = simple_strtoul(buf, NULL, 10);

	if(!strcmp(attr->attr.name, "keyboardbl_pwn"))
    {
        if(on)
         {
            gpio_direction_output(SABRESD_KYB_LEDEN, LOW);
            printk("keyboard_light control pin turn off!\r\n");
            imx6q_backlight_status[8] = LOW;
        }
        else
        {
            gpio_direction_output(SABRESD_KYB_LEDEN,HIGH);
            printk("keyboard_light control pin turn on!\r\n");
            imx6q_backlight_status[8] = HIGH;
        }
    } 
   
    return count;
 }


static DEVICE_ATTR(keyboardbl_pwn, 0666,  imx6q_backlight_read, imx6q_backlight_write);

static struct attribute *imx6q_backlight_sysfs_entries[] = {      
       
        &dev_attr_keyboardbl_pwn.attr,
        NULL,
};
static struct attribute_group imx6q_backlight_attr_group = {
	.name = NULL,
	.attrs = imx6q_backlight_sysfs_entries,
};

static int imx6q_backlight_control_probe(struct platform_device *pdev)
{
	imx6q_backlight_probe();
	return sysfs_create_group(&pdev->dev.kobj, &imx6q_backlight_attr_group);
}

static int imx6q_backlight_control_remove(struct platform_device *pdev)
{
	imx6q_backlight_remove();
	sysfs_remove_group(&pdev->dev.kobj, &imx6q_backlight_attr_group);
return 0;
}

static struct platform_driver imx6q_backlight_driver = {
	.probe   = imx6q_backlight_control_probe,
	.remove  = imx6q_backlight_control_remove,
	.driver  = {
		.name = "imx6q-backlight_control",
	},
};

static struct platform_device imx6q_backlight_device = {
	.name      = "imx6q-backlight_control",
	.id        = -1,
};

static int __devinit imx6q_backlight_init(void)
{
	int ret;
	printk("\n---------imx6q_backlight_init-----------\n");
	ret = platform_device_register(&imx6q_backlight_device);
	if(ret)
		printk("imx6q_backlight device register error\r\n");
	ret = platform_driver_register(&imx6q_backlight_driver);
	if(ret)
		printk("imx6q_backlight driver register error\r\n");
	return ret;
}
static void imx6q_backlight_exit(void)
{
	platform_driver_unregister(&imx6q_backlight_driver);
	printk("---------imx6q_backlight_exit-----------");
}
module_init(imx6q_backlight_init);
module_exit(imx6q_backlight_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("gxl");
MODULE_DESCRIPTION("imx6q platform gpio control driver");




