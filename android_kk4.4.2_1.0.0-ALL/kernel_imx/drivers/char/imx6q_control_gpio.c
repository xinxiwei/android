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


//#include <plat/gpio-cfg.h>

#define LOW                     (0)
#define HIGH                    (1)

#define IMX_GPIO_NR(bank, nr)	(((bank) - 1) * 32 + (nr))

#define SABRESD_GPS_EN	        IMX_GPIO_NR(3, 0)
#define SABRESD_WIFI_PWN	    IMX_GPIO_NR(6, 16)

/* #define SABRESD_YELLOW_LED      IMX_GPIO_NR(6, 15)
#define SABRESD_GREEN_LED       IMX_GPIO_NR(7, 8)
#define SABRESD_RED_LED         IMX_GPIO_NR(3, 19) */
//#define SABRESD_3V3_EN          IMX_GPIO_NR(6, 22)
#define SABRESD_3V3_EN          IMX_GPIO_NR(1, 9)


#define SABRESD_2V5_EN          IMX_GPIO_NR(6, 17)
#define SABRESD_1V2_EN          IMX_GPIO_NR(6, 18)
//#define SABRESD_KYB_LEDEN       IMX_GPIO_NR(6, 11)
#define SABRESD_FPGA_RESET      IMX_GPIO_NR(2, 5)

#define GPS_CTR          (1)
#define WIFI_CTR         (2)

/* #define YELLOW_LED_CTR   (13)
#define GREEN_LED_CTR    (14)
#define RED_LED_CTR      (15) */

#define FPGA_3V3_CTR     (6)
#define FPGA_2V5_CTR     (7)
#define FPGA_1V2_CTR     (8)
//#define KB_BL_CTR        (9)

#define FPGA_RESET       (10)
static struct class *gpioctr_class;  
struct cdev_imx6q {  
    struct cdev cdev;  
}; //建议用2.6的注册方法，2.4的已经离我们越来越远了。  
struct cdev_imx6q *imx6q_dev; 
static int  major =0; 

//static struct fasync_struct *fasync_queue;

static int imx6q_gpio_status[10] = { 0 };  //gpio控制的状态标志位


static int imx6q_gpio_open(struct inode *inode,struct file *file)
{
    //printk(KERN_INFO "get int funticon---> imx6q_gpio_open !\n");

    gpio_direction_input(SABRESD_GPS_EN);
    //gpio_direction_input(SABRESD_WIFI_PWN);
   /*  gpio_direction_input(SABRESD_YELLOW_LED);
    gpio_direction_input(SABRESD_GREEN_LED);
    gpio_direction_input(SABRESD_RED_LED); */
    gpio_direction_input(SABRESD_3V3_EN);
    gpio_direction_input(SABRESD_2V5_EN);
    gpio_direction_input(SABRESD_1V2_EN);
    //gpio_direction_input(SABRESD_KYB_LEDEN);
    gpio_direction_input(SABRESD_FPGA_RESET);
    return 0;
}

//static imx6q_gpio_release_fasync(int fd,struct file *filp,int mode)
//{
//
//      fasync_helper(fd,filp,mode,&fasync_queue);
//}

static void imx6q_gpio_close(struct inode* i_node,struct file* filp)  
{  
    printk(KERN_INFO "close init \n");  
    gpio_free(SABRESD_GPS_EN);      //释放  
    //gpio_free(SABRESD_WIFI_PWN);    //释放 
    //gpio_free(SABRESD_YELLOW_LED);
    //gpio_free(SABRESD_GREEN_LED);
    //gpio_free(SABRESD_RED_LED);
    gpio_free(SABRESD_3V3_EN);
    gpio_free(SABRESD_2V5_EN);
    gpio_free(SABRESD_1V2_EN);
    //gpio_free(SABRESD_KYB_LEDEN);
    gpio_free(SABRESD_FPGA_RESET);
    //imx6q_gpio_release_fasync(-1, filp, 0);
    return ;  
}  
 



//fasync方法的实现
//static int imx6q_gpio_fasync(int fd, struct file * filp, int on) 
//{
//    int retval;  

//    retval=fasync_helper(fd,filp,on,&fasync_queue);  
//    if(retval<0)
//    {
//        return retval;
//    }
//    return 0;
//}


 
static int imx6q_ioctl(struct file* filp,unsigned int cmd,unsigned long argv) 
{  
    int err = 0;
    //printk("imx6q_ioctl_gpio cmd = %d, argv = %d\r\n",cmd,argv); 
    switch(cmd){
        case GPS_CTR:
            if(argv>0){
                  err = gpio_direction_output(SABRESD_GPS_EN, HIGH);
                  //err = gpio_set_value(SABRESD_GPS_EN,0x01);
                  printk("GPS control pin turn off!\r\n");
               
            }else{
                  err = gpio_direction_output(SABRESD_GPS_EN, LOW);
                  //err = gpio_set_value(SABRESD_GPS_EN,0x00);
                  printk("GPS control pin turn on!\r\n");
            }
            break;
			
       case WIFI_CTR:
            if(argv>0){
                //err = gpio_direction_output(SABRESD_WIFI_PWN, HIGH);
                //err = gpio_set_value(SABRESD_WIFI_PWN,0x01);
                printk("WIFI control pin turn off!\r\n");
            }else{
                //err = gpio_direction_output(SABRESD_WIFI_PWN, LOW);
                //err = gpio_set_value(SABRESD_WIFI_PWN,0x00);
                printk("WIFI control pin turn on!\r\n");
            }
            break; 

			
			
      /*  case YELLOW_LED_CTR:
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
            break; */
	


	
     case FPGA_3V3_CTR:
            if(argv>0){
                err = gpio_direction_output(SABRESD_3V3_EN, HIGH);
                //err = gpio_set_value(SABRESD_3V3_EN,0x01);
                //printk("fpga_3v3 control pin turn on!\r\n");
            }else{
                err = gpio_direction_output(SABRESD_3V3_EN, LOW);
                //err = gpio_set_value(SABRESD_3V3_EN,0x00);
                //printk("fpga_3v3 control pin turn off!\r\n");
            }
            break;
			
     case FPGA_2V5_CTR:
            if(argv>0){
                err = gpio_direction_output(SABRESD_2V5_EN, LOW);
                //err = gpio_set_value(SABRESD_2V5_EN,0x01);
                //printk("fpga_2v5 control pin turn on!\r\n");
            }else{
                err = gpio_direction_output(SABRESD_2V5_EN, HIGH);
                //err = gpio_set_value(SABRESD_2V5_EN,0x00);
                //printk("fpga_2v5 control pin turn off!\r\n");
            }
            break;
			
     case FPGA_1V2_CTR:
            if(argv>0){
                err = gpio_direction_output(SABRESD_1V2_EN, HIGH);
                //err = gpio_set_value(SABRESD_1V2_EN,0x01);
                //printk("fpga_1v2 control pin turn on!\r\n");
            }else{
                err = gpio_direction_output(SABRESD_1V2_EN, LOW);
                //err = gpio_set_value(SABRESD_1V2_EN,0x00);
                //printk("fpga_1v2 control pin turn off!\r\n");
            }
            break;
			
    /*  case KB_BL_CTR:
            if(argv>0){
                err = gpio_direction_output(SABRESD_KYB_LEDEN, LOW);
                //err = gpio_set_value(SABRESD_KYB_LEDEN,0x01);
                printk("keyboard_backlight control pin turn off!\r\n");
            }else{
                err = gpio_direction_output(SABRESD_KYB_LEDEN, HIGH);
                //err = gpio_set_value(SABRESD_KYB_LEDEN,0x00);
                printk("keyboard_backlight control pin turn on!\r\n");
            }
            break; */
			
      case FPGA_RESET:
            if(argv>0){
                err = gpio_direction_output(SABRESD_FPGA_RESET, LOW);
                //err = gpio_set_value(SABRESD_FPGA_RESET,0x01);
                //printk("SABRESD_FPGA_RESET control pin turn on!\r\n");
            }else{
                err = gpio_direction_output(SABRESD_FPGA_RESET, HIGH);
                //err = gpio_set_value(SABRESD_FPGA_RESET,0x00);
                //printk("SABRESD_FPGA_RESET control pin turn off!\r\n");
            }
            break;  
			
        default:
            printk("cmd is not exist\n");
            return -1;
    }
    return err;
}  

/* file operations */  
struct file_operations fops={  
    .owner  = THIS_MODULE,  
    .open   = imx6q_gpio_open,  
    .unlocked_ioctl = imx6q_ioctl, // 特别注意从2.6.36以后ioctl已经移除，内核里面用unlocked_ioctl和compat_ioctl. 应用层不变，仍是ioctl调用。  
    .release= imx6q_gpio_close,  
    //.fasync  = imx6q_gpio_fasync,
};

static int imx6q_gpioctl_init(void)  
{  
    //printk(KERN_INFO "init .... \n");  
    dev_t dev_no;  
    int result,err; 

 
    err = alloc_chrdev_region(&dev_no,0,1,"devpwm_ctl"); //动态申请设备号  
    if(err<0)  
    {  
        printk(KERN_INFO "ERROR\n");  
        return err;  
    }  
    major = MAJOR(dev_no);  
    imx6q_dev = kmalloc(sizeof(struct cdev_imx6q),GFP_KERNEL);  
    if(!imx6q_dev)  
    {  
        result = -ENOMEM;  
        goto fail_malloc;  
    }  
    memset(imx6q_dev,0,sizeof(imx6q_dev));  
      
    /* major = register_chrdev(0, "led", &led_fops);*/

    cdev_init(&imx6q_dev->cdev,&fops); // 初始化cdev  
    imx6q_dev->cdev.owner = THIS_MODULE;  
    result = cdev_add(&imx6q_dev->cdev,dev_no,1); //加载设备  
    if(result <0)  
    {   printk(KERN_INFO "error\n");  
        goto fail_add;  
    }  


    gpioctr_class = class_create(THIS_MODULE,"imx6qpwrctr");  //在sys/class下创建sysfs文件  
    device_create(gpioctr_class,NULL,MKDEV(major,0),NULL,"devpwerctr"); //动态创建设备文件  /dev/devpwerctr， 以后不用手动创建了  
    return 0;  
fail_add:  
    kfree(imx6q_dev);  
fail_malloc:  
    unregister_chrdev_region(dev_no,1);  
    return result;  
  
}  

static void imx6q_dev_exit(void)  
{  
    dev_t dev_no=MKDEV(major,0);  
  
    unregister_chrdev_region(dev_no,1);  
    cdev_del(&imx6q_dev->cdev);  
    kfree(imx6q_dev);  
    device_destroy(gpioctr_class,dev_no);  
    class_destroy(gpioctr_class);  
    printk(KERN_INFO "exit........ \n");  
} 

static void imx6q_gpio_probe(void)
{
    int err;
	
    //printk("get into function -->imx6q_gpio_probe! \n");
    err = gpio_request(SABRESD_GPS_EN,"GPS_POWER_CTR");   
    if(err<0)  
    {  
        printk(KERN_INFO "GPS_POWER_CTR request faile \n");  
        return err;  
    }     
    //err = gpio_request(SABRESD_WIFI_PWN,"WIFI_POWER_CTR");   
    //if(err<0)  
    //{  
    //    printk(KERN_INFO "WIFI_POWER_CTR gpio request faile \n");  
    //    return err;  
    //}      
  /*   err = gpio_request(SABRESD_YELLOW_LED,"YELLOW_LED_CTR");   
    if(err<0)  
    {  
        printk(KERN_INFO "YELLOW_LED_CTR gpio request faile \n");  
        return err;  
    }    
    err = gpio_request(SABRESD_GREEN_LED,"GREEN_LED_CTR");   
    if(err<0)  
    {  
        printk(KERN_INFO "GREEN_LED_CTR gpio request faile \n");  
        return err;  
    }    
    err = gpio_request(SABRESD_RED_LED,"RED_LED_CTR");   
    if(err<0)  
    {  
        printk(KERN_INFO "RED_LED_CTR gpio request faile \n");  
        return err;  
    } */    
    err = gpio_request(SABRESD_3V3_EN,"FPGA_3V3_CTR");   
    if(err<0)  
    {  
        printk(KERN_INFO "FPGA_3V3_CTR gpio request faile \n");  
        return err;  
    } 
    gpio_direction_output(SABRESD_3V3_EN, LOW);
    err = gpio_request(SABRESD_2V5_EN,"FPGA_2V5_CTR");   
    if(err<0)  
    {  
        printk(KERN_INFO "FPGA_2V5_CTR gpio request faile \n");  
        return err;  
    } 
    gpio_direction_output(SABRESD_2V5_EN, HIGH);
    err = gpio_request(SABRESD_1V2_EN,"FPGA_1V2_CTR");   
    if(err<0)  
    {  
        printk(KERN_INFO "FPGA_1V2_CTR gpio request faile \n");  
        return err;  
    } 
    gpio_direction_output(SABRESD_1V2_EN, LOW);

   /*  err = gpio_request(SABRESD_KYB_LEDEN,"KYB_LED_CTR");   
    if(err<0)  
    {  
        printk(KERN_INFO "KYB_LED_CTR gpio request faile \n");  
        return err;  
    }  */
    err = gpio_request(SABRESD_FPGA_RESET,"FPGA_RESET");   
    if(err<0)  
    {  
        printk(KERN_INFO "FPGA_RESET gpio request faile \n");  
        return err;  
    } 
    imx6q_gpioctl_init();  
}



static  void  imx6q_gpio_remove(void)
{
    //gpio_free(SABRESD_WIFI_PWN);//释放GPIO引脚
    gpio_free(SABRESD_GPS_EN);
    //gpio_free(SABRESD_YELLOW_LED);
    //gpio_free(SABRESD_GREEN_LED);
    //gpio_free(SABRESD_RED_LED);
    gpio_free(SABRESD_3V3_EN);
    gpio_free(SABRESD_2V5_EN);
    gpio_free(SABRESD_1V2_EN);
    //gpio_free(SABRESD_KYB_LEDEN);
    gpio_free(SABRESD_FPGA_RESET);
    imx6q_dev_exit();
}

 static ssize_t  imx6q_gpio_read(struct device *dev, struct device_attribute *attr, char *buf)
{
     if(!strcmp(attr->attr.name, "wifi_pwn"))
     {
         if(imx6q_gpio_status[0] != 0)
             return   strlcpy(buf, "1\n", 3);
         else
             return   strlcpy(buf, "0\n", 3);
     }
     else if(!strcmp(attr->attr.name, "gps_pwn"))
     {
         if(imx6q_gpio_status[1] != 0)
             return   strlcpy(buf, "1\n", 3);
         else
             return  strlcpy(buf, "0\n", 3);
     }

     else if(!strcmp(attr->attr.name, "yellowled_pwn"))
     {
         if(imx6q_gpio_status[2] != 0)
             return  strlcpy(buf, "1\n", 3);
         else
             return  strlcpy(buf, "0\n", 3);
     }
     else if(!strcmp(attr->attr.name, "greenled_pwn"))
     {
         if(imx6q_gpio_status[3] != 0)
             return  strlcpy(buf, "1\n", 3);
         else
             return  strlcpy(buf, "0\n", 3);
     }
     else if(!strcmp(attr->attr.name, "redled_pwn"))
     {
         if(imx6q_gpio_status[4] != 0)
             return  strlcpy(buf, "1\n", 3);
         else
             return  strlcpy(buf, "0\n", 3);
     }
     else if(!strcmp(attr->attr.name, "fpga3v3_pwn"))
     {
         if(imx6q_gpio_status[5] != 0)
             return  strlcpy(buf, "1\n", 3);
         else
             return  strlcpy(buf, "0\n", 3);
     }
     else if(!strcmp(attr->attr.name, "fpga2v5_pwn"))
     {
         if(imx6q_gpio_status[6] != 0)
             return  strlcpy(buf, "1\n", 3);
         else
             return  strlcpy(buf, "0\n", 3);
     }     
     else if(!strcmp(attr->attr.name, "fpga1v2_pwn"))
     {
         if(imx6q_gpio_status[7] != 0)
             return  strlcpy(buf, "1\n", 3);
         else
             return  strlcpy(buf, "0\n", 3);
     }
     else if(!strcmp(attr->attr.name, "keyboardbl_pwn"))
     {
         if(imx6q_gpio_status[8] != 0)
             return  strlcpy(buf, "1\n", 3);
         else
             return  strlcpy(buf, "0\n", 3);
     }
     else if(!strcmp(attr->attr.name, "fpga_reset"))
     {
         if(imx6q_gpio_status[9] != 0)
             return  strlcpy(buf, "1\n", 3);
         else
             return  strlcpy(buf, "0\n", 3);
     }

     return   strlcpy(buf, "\n", 3);
}

static ssize_t imx6q_gpio_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned long on = simple_strtoul(buf, NULL, 10);

    
    if(!strcmp(attr->attr.name, "gps_pwn"))
    {
        if(on)
        {
             gpio_direction_output(SABRESD_GPS_EN, HIGH);
             printk("GPS control pin turn off!\r\n");
             imx6q_gpio_status[0] = HIGH;
        }
        else
        {
             gpio_direction_output(SABRESD_GPS_EN, LOW);
             printk("GPS control pin turn on!\r\n");
             imx6q_gpio_status[0] = LOW;
        }
    }
    else if(!strcmp(attr->attr.name, "wifi_pwn"))
    {
        if(on)
        {
            //gpio_direction_output(SABRESD_WIFI_PWN, HIGH);
            printk("WIFI control pin turn off!\r\n");
            imx6q_gpio_status[1] = HIGH;
        }
        else
        {
            //gpio_direction_output(SABRESD_WIFI_PWN,LOW);
            printk("WIFI control pin turn on!\r\n");
            imx6q_gpio_status[1] = LOW;
        }
    }

    /* else if(!strcmp(attr->attr.name, "yellowled_pwn"))
    {
        if(on)
        {
            gpio_direction_output(SABRESD_YELLOW_LED, LOW);
            printk("yellow_led control pin turn off!\r\n");
            imx6q_gpio_status[2] = LOW;
        }
        else
        {
            gpio_direction_output(SABRESD_YELLOW_LED,HIGH);
            printk("yellow_led control pin turn on!\r\n");
            imx6q_gpio_status[2] = HIGH;
        }
    }
    else if(!strcmp(attr->attr.name, "greenled_pwn"))
    {
        if(on)
        {
            gpio_direction_output(SABRESD_GREEN_LED, LOW);
            printk("green_led control pin turn off!\r\n");
            imx6q_gpio_status[3] = LOW;
        }
        else
        {
            gpio_direction_output(SABRESD_GREEN_LED,HIGH);
            printk("green_led control pin turn on!\r\n");
            imx6q_gpio_status[3] = HIGH;
        }
    }
    else if(!strcmp(attr->attr.name, "redled_pwn"))
    {
        if(on)
        {
            gpio_direction_output(SABRESD_RED_LED, LOW);
            printk("red_led control pin turn off!\r\n");
            imx6q_gpio_status[4] = LOW;
        }
        else
        {
            gpio_direction_output(SABRESD_RED_LED,HIGH);
            printk("red_led control pin turn on!\r\n");
            imx6q_gpio_status[4] = HIGH;
        }
    }
 */
    else if(!strcmp(attr->attr.name, "fpga3v3_pwn"))
    {
        if(on)
        {
            gpio_direction_output(SABRESD_3V3_EN, HIGH);
            //printk("fpga3v3 control pin turn off!\r\n");
            imx6q_gpio_status[5] = HIGH;
        }
        else
        {
            gpio_direction_output(SABRESD_3V3_EN,LOW);
           // printk("fpga3v3 control pin turn on!\r\n");
            imx6q_gpio_status[5] = LOW;
        }
    }
    else if(!strcmp(attr->attr.name, "fpga2v5_pwn"))
    {
        if(on)
        {
            gpio_direction_output(SABRESD_2V5_EN, LOW);
            //printk("fpga2v5 control pin turn off!\r\n");
            imx6q_gpio_status[6] = LOW;
        }
        else
        {
            gpio_direction_output(SABRESD_2V5_EN,HIGH);
            //printk("fpga2v5 control pin turn on!\r\n");
            imx6q_gpio_status[6] = HIGH;
        }
    }
    else if(!strcmp(attr->attr.name, "fpga1v2_pwn"))
    {
        if(on)
        {
            gpio_direction_output(SABRESD_1V2_EN, HIGH);
            //printk("WIFI control pin turn off!\r\n");
            imx6q_gpio_status[7] = HIGH;
        }
        else
        {
            gpio_direction_output(SABRESD_1V2_EN,LOW);
            //printk("WIFI control pin turn on!\r\n");
            imx6q_gpio_status[7] = LOW;
        }
    }
   /*  else if(!strcmp(attr->attr.name, "keyboardbl_pwn"))
    {
        if(on)
        {
            gpio_direction_output(SABRESD_KYB_LEDEN, LOW);
            printk("keyboard_backlight control pin turn off!\r\n");
            imx6q_gpio_status[8] = LOW;
        }
        else
        {
            gpio_direction_output(SABRESD_KYB_LEDEN,HIGH);
            printk("keyboard_backlight control pin turn on!\r\n");
            imx6q_gpio_status[8] = HIGH;
        }
    } */
   
     else if(!strcmp(attr->attr.name, "fpga_reset"))
    {
        if(on)
        {
            gpio_direction_output(SABRESD_FPGA_RESET, LOW);
           // printk("fpga_reset control pin turn on!\r\n");
            imx6q_gpio_status[9] = LOW;
        }
        else
        {
            gpio_direction_output(SABRESD_FPGA_RESET,HIGH);
           // printk("fpga_reset control pin turn off!\r\n");
            imx6q_gpio_status[9] = HIGH;
        }
    }
    return count;
 }
static DEVICE_ATTR(gps_pwn,        0666,  imx6q_gpio_read, imx6q_gpio_write);
static DEVICE_ATTR(wifi_pwn,       0666,  imx6q_gpio_read, imx6q_gpio_write);
/* static DEVICE_ATTR(yellowled_pwn,  0666,  imx6q_gpio_read, imx6q_gpio_write);
static DEVICE_ATTR(greenled_pwn,   0666,  imx6q_gpio_read, imx6q_gpio_write);
static DEVICE_ATTR(redled_pwn,     0666,  imx6q_gpio_read, imx6q_gpio_write); */
static DEVICE_ATTR(fpga3v3_pwn,    0666,  imx6q_gpio_read, imx6q_gpio_write);
static DEVICE_ATTR(fpga2v5_pwn,    0666,  imx6q_gpio_read, imx6q_gpio_write);
static DEVICE_ATTR(fpga1v2_pwn,    0666,  imx6q_gpio_read, imx6q_gpio_write);
static DEVICE_ATTR(keyboardbl_pwn, 0666,  imx6q_gpio_read, imx6q_gpio_write);
static DEVICE_ATTR(fpga_reset    , 0666,  imx6q_gpio_read, imx6q_gpio_write);

static struct attribute *imx6q_gpio_sysfs_entries[] = {
	&dev_attr_gps_pwn.attr,
        &dev_attr_wifi_pwn.attr,       
     /*    &dev_attr_yellowled_pwn.attr,
	&dev_attr_greenled_pwn.attr,
        &dev_attr_redled_pwn.attr, */
        &dev_attr_fpga3v3_pwn.attr,
        &dev_attr_fpga2v5_pwn.attr,
        &dev_attr_fpga1v2_pwn.attr,
        &dev_attr_keyboardbl_pwn.attr,
        &dev_attr_fpga_reset.attr,
        NULL,
};
static struct attribute_group imx6q_gpio_attr_group = {
	.name = NULL,
	.attrs = imx6q_gpio_sysfs_entries,
};

static int imx6q_gpio_control_probe(struct platform_device *pdev)
{
	imx6q_gpio_probe();
	return sysfs_create_group(&pdev->dev.kobj, &imx6q_gpio_attr_group);
}

static int imx6q_gpio_control_remove(struct platform_device *pdev)
{
	imx6q_gpio_remove();
	sysfs_remove_group(&pdev->dev.kobj, &imx6q_gpio_attr_group);
return 0;
}

static struct platform_driver imx6q_gpio_driver = {
	.probe   = imx6q_gpio_control_probe,
	.remove  = imx6q_gpio_control_remove,
	.driver  = {
		.name = "imx6q-gpio_control",
	},
};

static struct platform_device imx6q_gpio_device = {
	.name      = "imx6q-gpio_control",
	.id        = -1,
};

static int __devinit imx6q_gpio_init(void)
{
	int ret;
	printk("---------imx6q_gpio_init-----------\n");
	ret = platform_device_register(&imx6q_gpio_device);
	if(ret)
		printk("imx6q_gpio device register error\r\n");
	ret = platform_driver_register(&imx6q_gpio_driver);
	if(ret)
		printk("imx6q_gpio driver register error\r\n");
	return ret;
}
static void imx6q_gpio_exit(void)
{
	platform_driver_unregister(&imx6q_gpio_driver);
	printk("---------imx6q_gpio_exit-----------");
}
module_init(imx6q_gpio_init);
module_exit(imx6q_gpio_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("gxl");
MODULE_DESCRIPTION("imx6q platform gpio control driver");




