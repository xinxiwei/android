/*
 *
 * Date:2016-8-9
 * Author:lhx
 * Description: this file call the linux driver .
 *
 *
 */

#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <hardware/log.h>
#include <sys/ioctl.h>
#include <hardware/beep.h>
#include <hardware/errorcode.h>

static int beepfd = -1;
int BeepValue = 2000;

int BeepSetFcy(int fcy)
{
		int ret;
		struct input_event event;
		event.type = EV_SND;     //event type 蜂鸣器
		event.code = SND_TONE;   //sound type 声音类型
		event.value = fcy;      //modify value change beep  (0 --->turn off beep)
		//LOGD("write to beep. fcy = %d",fcy);
		LOGD("beepfd = %d",beepfd);
		ret = write(beepfd, &event, sizeof(struct input_event));
		return ret;
}

int BeepOpen()
{
	if(beepfd == 0 || beepfd == -1)
	{
		beepfd = open("/dev/input/event2", O_RDWR);
		if ( beepfd == -1 || beepfd == 0)
		{
			beepfd = open("/dev/input/event2", O_RDWR);
			if ( beepfd == -1 )
			{
				LOGD("打开beep声设备文件 /dev/input/event2 失败 .");
				return -1;
			}
		}
	}
	LOGD( "打开beep声设备文件 /dev/input/event2 成功, beepfd = %d",beepfd);
	return 0;
}

int BeepClose()
{
    if(close(beepfd) < 0)
	{
		LOGD( "关闭beep声设备文件 /dev/input/event2 失败");
		return -1;
	}
    LOGD( "关闭beep声设备文件 /dev/input/event2 成功, beepfd = %d",beepfd);
    return 0;
}

int BeepOff()
{
	if(BeepSetFcy(0) < 0)
	{
		LOGE( "In BeepOff BeepSetFcy(0) error.");
		return -1;
	}
	return 0;
}


int BeepOn(int count)
{
    LOGE("BeepOn count = %d\n",count);
	if(BeepSetFcy(count) < 0)
	{
		LOGE( "In BeepOn BeepSetFcy(count) error.");
		return -1;
	}
	return 0;
}
