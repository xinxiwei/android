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
		LOGD("write to beep. fcy = %d",fcy);
		ret = write(beepfd, &event, sizeof(struct input_event));
		return ret;
}

 int BeepOpen()
 {
	 if ((beepfd = open("/dev/input/event1", O_RDWR)) < 0)
	 {
		 LOGD("Beep open fail.");
		 return -1;
	 }
	 LOGD( "Beep open /dev/input/event1 success. fd = %d",beepfd);
	 return 0;
 }

 int BeepClose()
{
	LOGD( "Beep close. fd = %d",beepfd);
    if(close(beepfd) < 0)
	{
		LOGE( "In BeepClose close(beepfd) error.");
		return -1;
	}
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
	if(BeepSetFcy(count) < 0)
	{
		LOGE( "In BeepOn BeepSetFcy(count) error.");
		return -1;
	}
	return 0;
}








