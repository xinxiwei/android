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

#define SABRESD_YELLOW_LED      IMX_GPIO_NR(6, 15)
#define SABRESD_GREEN_LED       IMX_GPIO_NR(7, 8)
#define SABRESD_RED_LED         IMX_GPIO_NR(3, 19)
#define BEEP_SOUND_CTR          IMX_GPIO_NR(2, 3) 

#define YELLOW_LED_CTR   (13)
#define GREEN_LED_CTR    (14)
#define RED_LED_CTR      (15)


static struct class *lightctr_class;  
struct cdev_imx6q_light {  
    struct cdev cdev;  
}; //建议用2.6的注册方法，2.4的已经离我们越来越远了。  
struct cdev_imx6q_light *imx6q_dev_light; 
static int  major =0; 

static int imx6q_light_status[4] = { 0 };  //gpio控制的状态标志位


static int imx6q_light_open(struct inode *inode,struct file *file)
{
    printk(KERN_INFO "get int funticon---> imx6q_light_open !\n");
   
    return 0;
}

static void imx6q_light_close(struct inode* i_node,struct file* filp)  
{  
    printk(KERN_INFO "close init \n");  
   
    return ;  
}  

static int imx6q_ioctl(struct file* filp,unsigned int cmd,unsigned long argv) 
{  
    int err = 0;
    printk("imx6q_ioctl_light cmd = %d, argv = %d\r\n",cmd,argv); 
  
    switch(cmd){
       case YELLOW_LED_CTR:
            if(argv>0){
                err = gpio_direction_output(SABRESD_YELLOW_LED, LOW);
                printk("yellow_led control pin turn on!\r\n");
            }else{
                err = gpio_direction_output(SABRESD_YELLOW_LED, HIGH);
                printk("yellow_led control pin turn off!\r\n");
            }
            break;
      case GREEN_LED_CTR:
            if(argv>0){
                err = gpio_direction_output(SABRESD_GREEN_LED, LOW);
                printk("green_led control pin turn on!\r\n");
            }else{
                err = gpio_direction_output(SABRESD_GREEN_LED, HIGH);
                printk("green_led control pin turn off!\r\n");
            }
            break;
      case RED_LED_CTR:
            if(argv>0){
                err = gpio_direction_output(SABRESD_RED_LED, LOW);
                printk("red_led control pin turn on!\r\n");
            }else{
                err = gpio_direction_output(SABRESD_RED_LED, HIGH);
                printk("red_led control pin turn off!\r\n");
            }
            break; 
        default:
            printk("cmd is not exist\n");
            return -1;
    }
    return err;
}  


/* file operations */  
struct file_operations fops_light={  
    .owner  = THIS_MODULE,  
    .open   = imx6q_light_open,  
    .unlocked_ioctl = imx6q_ioctl, // 特别注意从2.6.36以后ioctl已经移除，内核里面用unlocked_ioctl和compat_ioctl. 应用层不变，仍是ioctl调用。  
    .release= imx6q_light_close, 
};

static int imx6q_lightctl_init(void)  
{  
    printk(KERN_INFO "init .... \n");  
    dev_t dev_no;  
    int result,err; 
 
    err = alloc_chrdev_region(&dev_no,0,1,"dev_light"); //动态申请设备号  
    if(err<0)  
    {  
        printk(KERN_INFO "ERROR\n");  
        return err;  
    }  
    major = MAJOR(dev_no);  
    imx6q_dev_light = kmalloc(sizeof(struct cdev_imx6q_light),GFP_KERNEL);  
    if(!imx6q_dev_light)  
    {  
        result = -ENOMEM;  
        goto fail_malloc;  
    }  
    memset(imx6q_dev_light,0,sizeof(imx6q_dev_light));  
      
    cdev_init(&imx6q_dev_light->cdev,&fops_light); // 初始化cdev  
    imx6q_dev_light->cdev.owner = THIS_MODULE;  
    result = cdev_add(&imx6q_dev_light->cdev,dev_no,1); //加载设备  
    if(result <0)  
    {   printk(KERN_INFO "error\n");  
        goto fail_add;  
    }  

    lightctr_class = class_create(THIS_MODULE,"imx6qlightctr");  //在sys/class下创建sysfs文件  
    device_create(lightctr_class,NULL,MKDEV(major,0),NULL,"devlightctr"); //动态创建设备文件  /dev/devlightctr， 以后不用手动创建了  
    return 0;  
fail_add:  
    kfree(imx6q_dev_light);  
fail_malloc:  
    unregister_chrdev_region(dev_no,1);  
    return result;    
}  

static void imx6q_dev_light_exit(void)  
{  
    dev_t dev_no=MKDEV(major,0);  
  
    unregister_chrdev_region(dev_no,1);  
    cdev_del(&imx6q_dev_light->cdev);  
    kfree(imx6q_dev_light);  
    device_destroy(lightctr_class,dev_no);  
    class_destroy(lightctr_class);  
    printk(KERN_INFO "exit........ \n");  
} 

static void imx6q_light_probe(void)
{
    int err;
      
    printk("\nget into function -->imx6q_light_probe! \n");    
     
    err = gpio_request(SABRESD_YELLOW_LED,"YELLOW_LED_CTR");   
    if(err<0)  
    {  
        printk(KERN_INFO "YELLOW_LED_CTR gpio request faile \n");  
        return ;  
    }    
    err = gpio_request(SABRESD_GREEN_LED,"GREEN_LED_CTR");   
    if(err<0)  
    {  
        printk(KERN_INFO "GREEN_LED_CTR gpio request faile \n");  
        return ;  
    }    
    err = gpio_request(SABRESD_RED_LED,"RED_LED_CTR");   
    if(err<0)  
    {  
        printk(KERN_INFO "RED_LED_CTR gpio request faile \n");  
        return ;  
    }    
   
	gpio_direction_input(SABRESD_YELLOW_LED);
    gpio_direction_input(SABRESD_GREEN_LED);
    gpio_direction_input(SABRESD_RED_LED);
    imx6q_lightctl_init();  
	
	/////调试开机蜂鸣器一直响的问题
	err = gpio_request(BEEP_SOUND_CTR, "beeper ctr");
	if(err<0)  
    {  
        printk(KERN_INFO "BEEP_SOUND_CTR gpio request faile \n");  
        return ;  
    } 
	gpio_direction_output(BEEP_SOUND_CTR, LOW); //等价于 gpio_set_value(BEEP_SOUND_CTR, LOW);
}



static  void  imx6q_light_remove(void)
{
    gpio_free(SABRESD_YELLOW_LED);
    gpio_free(SABRESD_GREEN_LED);
    gpio_free(SABRESD_RED_LED);
    imx6q_dev_light_exit();
}

 static ssize_t  imx6q_light_read(struct device *dev, struct device_attribute *attr, char *buf)
{
     if(!strcmp(attr->attr.name, "yellowled_pwn"))
     {
         if(imx6q_light_status[0] != 0)
             return  strlcpy(buf, "1\n", 3);
         else
             return  strlcpy(buf, "0\n", 3);
     }
     else if(!strcmp(attr->attr.name, "greenled_pwn"))
     {
         if(imx6q_light_status[1] != 0)
             return  strlcpy(buf, "1\n", 3);
         else
             return  strlcpy(buf, "0\n", 3);
     }
     else if(!strcmp(attr->attr.name, "redled_pwn"))
     {
         if(imx6q_light_status[2] != 0)
             return  strlcpy(buf, "1\n", 3);
         else
             return  strlcpy(buf, "0\n", 3);
     }	
     return   strlcpy(buf, "\n", 3);
}

static ssize_t imx6q_light_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned long on = simple_strtoul(buf, NULL, 10);

    if(!strcmp(attr->attr.name, "yellowled_pwn"))
    {
        if(on)
        {
            gpio_direction_output(SABRESD_YELLOW_LED, LOW);
            printk("yellow_led control pin turn off!\r\n");
            imx6q_light_status[2] = LOW;
        }
        else
        {
            gpio_direction_output(SABRESD_YELLOW_LED,HIGH);
            printk("yellow_led control pin turn on!\r\n");
            imx6q_light_status[2] = HIGH;
        }
    }
    else if(!strcmp(attr->attr.name, "greenled_pwn"))
    {
        if(on)
        {
            gpio_direction_output(SABRESD_GREEN_LED, LOW);
            printk("green_led control pin turn off!\r\n");
            imx6q_light_status[3] = LOW;
        }
        else
        {
            gpio_direction_output(SABRESD_GREEN_LED,HIGH);
            printk("green_led control pin turn on!\r\n");
            imx6q_light_status[3] = HIGH;
        }
    }
    else if(!strcmp(attr->attr.name, "redled_pwn"))
    {
        if(on)
        {
            gpio_direction_output(SABRESD_RED_LED, LOW);
            printk("red_led control pin turn off!\r\n");
            imx6q_light_status[4] = LOW;
        }
        else
        {
            gpio_direction_output(SABRESD_RED_LED,HIGH);
            printk("red_led control pin turn on!\r\n");
            imx6q_light_status[4] = HIGH;
        }
    }
    return count;
 }

static DEVICE_ATTR(yellowled_pwn,  0666,  imx6q_light_read, imx6q_light_write);
static DEVICE_ATTR(greenled_pwn,   0666,  imx6q_light_read, imx6q_light_write);
static DEVICE_ATTR(redled_pwn,     0666,  imx6q_light_read, imx6q_light_write);

static struct attribute *imx6q_light_sysfs_entries[] = {      
        &dev_attr_yellowled_pwn.attr,
	    &dev_attr_greenled_pwn.attr,
        &dev_attr_redled_pwn.attr,
        NULL,
};
static struct attribute_group imx6q_light_attr_group = {
	.name = NULL,
	.attrs = imx6q_light_sysfs_entries,
};

static int imx6q_light_control_probe(struct platform_device *pdev)
{
	imx6q_light_probe();
	return sysfs_create_group(&pdev->dev.kobj, &imx6q_light_attr_group);
}

static int imx6q_light_control_remove(struct platform_device *pdev)
{
	imx6q_light_remove();
	sysfs_remove_group(&pdev->dev.kobj, &imx6q_light_attr_group);
return 0;
}

static struct platform_driver imx6q_light_driver = {
	.probe   = imx6q_light_control_probe,
	.remove  = imx6q_light_control_remove,
	.driver  = {
		.name = "imx6q-light_control",
	},
};

static struct platform_device imx6q_light_device = {
	.name      = "imx6q-light_control",
	.id        = -1,
};

static int __devinit imx6q_light_init(void)
{
	int ret;
	printk("\n---------imx6q_light_init-----------\n");
	ret = platform_device_register(&imx6q_light_device);
	if(ret)
		printk("imx6q_light device register error\r\n");
	ret = platform_driver_register(&imx6q_light_driver);
	if(ret)
		printk("imx6q_light driver register error\r\n");
	return ret;
}
static void imx6q_light_exit(void)
{
	platform_driver_unregister(&imx6q_light_driver);
	printk("---------imx6q_light_exit-----------");
}
module_init(imx6q_light_init);
module_exit(imx6q_light_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("gxl");
MODULE_DESCRIPTION("imx6q platform gpio control driver");




