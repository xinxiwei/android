#define LOG_TAG "keysound"
#include <hardware/hardware.h>
#include <hardware/imx6q_keysound.h>
#include <errno.h>
#include <cutils/log.h>
#include <cutils/atomic.h>
#include <sys/ioctl.h>
#include <hardware/log.h>
#include <hardware/gpio.h>
#include <hardware/errorcode.h>
#include <hardware/beep.h>
#include <fcntl.h>
#include <stdbool.h>
#include <sys/timeb.h>
#include <linux/input.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/wait.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/timeb.h>
#include <time.h>


#define DEVICE_NAME "/dev/input/event0"
#define MODULE_NAME "xxxxx"
#define MODULE_AUTHOR "xxx@126.com"


int keys_fd =-1;
int backlight_fd = -1;

volatile int soundOnOff_flag =0;
volatile int backlightOnOff_flag =0;
static int count = 0;

int KEY_ESC_CODE = 59;
int KEY_CANCEL_CODE = 223;
static int KEY_LIGHT =9;

pthread_t th_key,sen_th;

/*�豸�򿪺͹رսӿ�*/
static int keypad_device_open(const struct hw_module_t* module,const char* name,struct hw_device_t** device);
static int keypad_device_close(struct hw_device_t* device);
/*�豸���ʽӿ�*/
static int keypad_device_enable(struct keyctl_device_t* device,int key);
static int keypad_device_disable(struct keyctl_device_t* device,int key);

static int keypad_set_val(struct keyctl_device_t* dev,int val);
static int keypad_get_val(struct keyctl_device_t* dev,int* val);
/*ģ�鷽����*/
static struct hw_module_methods_t key_module_methods={
   open: keypad_device_open
};
/*ģ��ʵ������*/
struct keyctl_module_t HAL_MODULE_INFO_SYM ={
    common:{
        tag: HARDWARE_MODULE_TAG,
        version_major: 1,
        version_minor: 0,
        id: KEYCTL_HARDWARE_MODULE_ID,
        name: MODULE_NAME,
        author: MODULE_AUTHOR,
        methods: &key_module_methods,
    }
};

char* log_time(  )
{
	struct  tm      *ptm;
	struct  timeb   stTimeb;
	static  char    szTime[19];
	ftime( &stTimeb );
	ptm = localtime( &stTimeb.time );
	sprintf( szTime ,  "%02d-%02d %02d:%02d:%02d.%03d" ,  ptm->tm_mon+1 ,  ptm->tm_mday ,  ptm->tm_hour ,  ptm->tm_min ,  ptm->tm_sec ,  stTimeb.millitm );
	szTime[18] = 0;
	return szTime;
}

int backlight_open()
{
	 if ((backlight_fd = open("/dev/devbacklightctr", O_RDWR)) < 0)
	 {
		 ALOGD("backlight open fail.");
		 return -1;
	 }
	 ALOGD( "backlight open /dev/devbacklightctr success. fd = %d",backlight_fd);
	 return 0;
}

void KeyOnOff(void *arg)
{
    ALOGD ("KeyOnOff"); 
    struct input_event t;
    while (1)//�߳�ѭ����
    {
        if(read(keys_fd, &t, sizeof(t)) == sizeof(t)) 
        {
            if (t.type == EV_KEY)  // 
            {           
                if(t.value == 0)// key up 
                {     
                    if(soundOnOff_flag == 1)
                    {
                        BeepOn(4000);//�����һ���������£�������������
                        usleep(100000); //ֻ��100ms
                        BeepOn(0);//ֹͣ��������                    
                    }                                  
                }                     
            }        
            if(soundOnOff_flag == 0)
            {
                ALOGD ("soundOnOff_flag=0.th_key thread exit");                    
                return ;
            } 
        }
   }
}

volatile int flag = 0;

void lightOnOff(void *arg)
{
    ALOGD ("lightOnOff"); 
    struct input_event t;    
    int num = 5;
    int key = *(int *)arg;
    ALOGD("lightOnOff_key = %d\n" , key);
    
    while (1)//�߳�ѭ����
    {
        if(backlightOnOff_flag == 2)
        {
            if(read(keys_fd, &t, sizeof(t)) == sizeof(t)) 
            {
                if (t.type == EV_KEY)
                {  
                    if(t.value == 1)// key down
                    {    
                        ALOGD ("sen_th key= %d %s\n", t.code,(t.value) ? "Pressed" : "Released");
                        flag =1;                     
                        ioctl(backlight_fd,KEY_LIGHT,1);
                        ALOGD("���� [%s]\n" ,  log_time( ) );
                        usleep(500000);
                        ioctl(backlight_fd,KEY_LIGHT,0); 
                        ALOGD("�ص� [%s]\n" ,  log_time( ) );
                    }
                    
                    /* if(t.value == 0)// key up
                    {  
                        if(flag)
                        {
                            usleep(500000);
                            flag = 0;
                        }
                        if(!flag)
                        {
                            ioctl(backlight_fd,KEY_LIGHT,0);  
                            ALOGD("�ص� [%s]\n" ,  log_time( ) );
                        }
                    } */
                }
            }           
        }
        /* else{
            ALOGD("lightOnOff--exit0000" );
            return;
        } */
        
        if( soundOnOff_flag == 0 && backlightOnOff_flag == 0)
        {
            ALOGD("lightOnOff--exit1111" );
            return ;
        }            
   }
}

static int keypad_device_open(const struct hw_module_t* module,const char* name,struct hw_device_t** device)
{
    struct keyctl_device_t* dev;
    int res = 0;
    dev = (struct keyctl_device_t*)malloc(sizeof(struct keyctl_device_t));
    if(!dev){
        ALOGD("keyctr Stub: failed to alloc space");
        return -EFAULT;
    }
    memset(dev,0,sizeof(struct keyctl_device_t));
    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (hw_module_t*)module;
    dev->common.close = keypad_device_close;
	dev->key_enable = keypad_device_enable;
	dev->key_disable = keypad_device_disable;
	
    dev->set_key_val = keypad_set_val;
    dev->get_key_val = keypad_get_val; 
    *device =&(dev->common);
    
    dev->fd = open (DEVICE_NAME, O_RDONLY);
    keys_fd = dev->fd;    
    if (keys_fd <= 0)
    {
        ALOGD ("open /dev/input/event0 device error!");
        if(close(keys_fd) <0)
        {
            ALOGD ("close /dev/input/event0 error!");
        }
        return -1;
    }    
    ALOGD("open /dev/input/event0 success. fd=%d",keys_fd); 
    if(backlight_open()<0)
    {
        ALOGD("backlight_open() error.");
    }
    return 0;
}



static int keypad_device_enable(struct keyctl_device_t* device,int key)//���豸 
{ 
    ALOGD ("keypad_device_enable====start ,key = %d",key);
    int res = 0;
    int status_key = key;
    if(key == 3) //����
    {
        ALOGD("enable beep");
        soundOnOff_flag = 1;
        if(BeepOpen()<0)
        {
            ALOGD("BeepOpen() error.");
        }
        res=pthread_create(&th_key, NULL, (void *)KeyOnOff, (void *)0);//���������߳�
        if(res != 0)
        {
            ALOGD("Fail to create a new thread th_key");
            return -1;
        }                
    }else 
    {
        if(key == 0)
        {
            ALOGD ("���̱��⣺ һֱ�ر�");
            ioctl(backlight_fd,KEY_LIGHT,0);
            /* if(close(backlight_fd) <0 )
            {
                ALOGD("close backlight_fd device error.");
                return -1;
            }  */          
            backlightOnOff_flag = 0;
        }
        if(key == 1)
        {
            ALOGD ("���̱��⣺ һֱ��");
            backlightOnOff_flag = 1;
            ioctl(backlight_fd,KEY_LIGHT,1);
        }
        
        if(key == 2)
        {
            ALOGD ("���̱��⣺���ܸй�");
            ioctl(backlight_fd,KEY_LIGHT,0);
            res=pthread_create(&sen_th, NULL, (void *)lightOnOff, (void*)&status_key);//���������߳�
            if(res != 0)
            {
                ALOGD("Fail to create a new thread sen_th");
                return -1;
            }
            backlightOnOff_flag = 2;
        }        
    }
    
    ALOGD ("keypad_device_enable====end");
	return 0;
}


static int keypad_device_disable(struct keyctl_device_t* device,int key)//�ر��豸 
{ 
    ALOGD ("keypad_device_disable===start");
    if(key == 3)
    {
        ALOGD("disable beep");
        soundOnOff_flag = 0;
        if(BeepClose() <0)
        {
            ALOGD("BeepClose() error.");
            return -1;
        }
    }	
   if( soundOnOff_flag == 0 && backlightOnOff_flag == 0)
    {
       if(close(keys_fd) <0 )
        {
            ALOGD("close key  device error.");
            return -1;
        }
    }
    ALOGD ("keypad_device_disable===end");
    return 0;
}

static int keypad_set_val(struct keyctl_device_t* dev,int val){
    ALOGD("keyctr Stub: set value %d to device.",val);
    write(dev->fd,&val,sizeof(val)); 
    return 0;
}
static int keypad_get_val(struct keyctl_device_t* dev,int* val){
    if(!val){
        ALOGE("keyctr Stub:?error?val?pointer");
        return -EFAULT;
    }
    read(dev->fd,val,sizeof(*val));
    ALOGD("keyctr Stub: get value %d from device",*val);
    return 0;
}

static int keypad_device_close(struct hw_device_t* device){ //�ر��豸 
    struct keyctl_device_t* devkey_device = (struct keyctl_device_t*)device;	
    if(devkey_device){		
        close(devkey_device->fd);
        free(devkey_device);
    }
    return 0;
}

