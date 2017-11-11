/*
 * Copyright (C) 2004-2007, 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2008 Juergen Beisert
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation
 * 51 Franklin Street, Fifth Floor
 * Boston, MA  02110-1301, USA.
 */

 
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>


#include <mach/dma.h>
#include <mach/spi.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/mutex.h>

#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/errno.h>
//#include <linux/signal.h>

#include <linux/sched.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/delay.h>
#include <linux/cdev.h>
#include <linux/poll.h>


#define MXC_CSPIRXDATA		0x00
#define MXC_CSPITXDATA		0x04
#define MXC_CSPICTRL		0x08
#define MXC_CSPIINT		0x0c
#define MXC_RESET		0x1c

#define MX3_CSPISTAT		0x14
#define MX3_CSPISTAT_RR		(1 << 3)

/* generic defines to abstract from the different register layouts */
#define MXC_INT_RR	(1 << 0) /* Receive data ready interrupt */
#define MXC_INT_TE	(1 << 1) /* Transmit FIFO empty interrupt */
#define MXC_INT_TDR (1 << 2)
#define MXC_INT_RDR (1 << 3)
#define MXC_INT_RF  (1 << 5)
#define SPI_BUF_SIZE PAGE_SIZE	



#define  SPIDEV_IOC_SET_RX_DMASIZE     (0x01)
#define  SPIDEV_IOC_REQ_RXBUFS         (0x02)
#define  SPIDEV_IOC_QUERY_RXBUF        (0x03)
#define  SPIDEV_IOC_ENQUEUE_RXBUF      (0x04) 
#define  SPIDEV_IOC_DEQUEUE_RXBUF      (0x05)
#define  SPIDEV_IOC_RXSTREAMON         (0x06)   
#define  SPIDEV_IOC_RXSTREAMOFF        (0x07) 
#define  SPIDEV_IOC_QUEUE_STATE        (0X08)

#define DRIVER_NAME "spislavedev"

#define SPIDEV_IMX_MAJOR    152//153
#define N_SPIDEV_MINORS     32

#define IMX_GPIO_NR(bank, nr)	(((bank) - 1) * 32 + (nr))

#define DMA_BUF_SZIE   10 //10

static DECLARE_BITMAP(minors, N_SPIDEV_MINORS);

static struct class *spidev_imx_class;
struct spi_imx_config {
	unsigned int speed_hz;
	unsigned int bpw;
	unsigned int mode;
	u8 cs;
};

enum spi_imx_devtype {

    SPI_IMX_VER_2_3_SLAVE,
};


struct spidev_buffer{
    u32  offset;
    u32  index;
    u8   flags;
    u32  length;
};

typedef struct spidev_data_frame {
  u32 paddress;   
  u32 *vaddress;  
  //int index;    
  //struct spidev_buffer  buffer;
  struct scatterlist	rx_sgl;
  struct list_head      queue;
}spidev_data_frame_t;


struct spi_imx_data;

struct spi_imx_devtype_data {
	void (*intctrl)(struct spi_imx_data *, int);
	int (*config)(struct spi_imx_data *, struct spi_imx_config *);
	void (*trigger)(struct spi_imx_data *);
	int (*rx_available)(struct spi_imx_data *);
	void (*reset)(struct spi_imx_data *);
	unsigned int fifosize;
};

struct spi_imx_data {
	struct platform_device *pdev;
    spinlock_t lock;
    dev_t devt;
	struct completion xfer_done;
	void *base;
	int irq;
	struct clk *clk;
	unsigned long spi_clk;
    spinlock_t  spi_lock;
    int bus_num;
	int *chipselect;

	int count;

	void *rx_buf;
	const void *tx_buf;

	int txfifo;                       /* number of words pushed in tx FIFO */
    int master_mode; 		  /*slave or master*/
	u16 rx_threshold;  
	u16 tx_threshold;   	
	u16 dma_req_rx;
	u16 dma_req_tx;							
	u16 bpw;
	struct dma_chan	*dma_chan_rx;
	struct dma_chan	*dma_chan_tx;
	struct scatterlist	rx_sg;
   	struct scatterlist	tx_sg;
	struct dma_async_tx_descriptor *tx_desc;
	struct dma_async_tx_descriptor *rx_desc;
	struct device *dev;

    u8 work_queue_state;
    u8 ready_queue_state;
    resource_size_t mapbase; 
    void __iomem *membase; 
         
    struct fasync_struct *fasync_queue;
       
    struct list_head    device_entry; 

    spidev_data_frame_t rxsegnums[DMA_BUF_SZIE]; 
    struct list_head ready_q; 
  	//struct list_head done_q;  
  	struct list_head working_q;
       
    u32 rxdma_buf_size ;
	u32 num_rxdma_bufs ;
    u16 rx_dma_size;
	struct spi_imx_devtype_data devtype_data;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static int spidev_rxstreamon(struct spi_imx_data *spi_imx);
static void spi_imx_dma_rx_callback(void * data);

static int  spidev_alloc_mem_rxframe(struct spi_imx_data *spi_devdata );
static int  spidev_init_rxsegnums_buf(struct spi_imx_data *spi_devdata);
static void spidev_free_mem_rxframe(struct spi_imx_data *spi_imx);
static void spidev_deinit_rxframe_buf(struct spi_imx_data  *spi_devdata);



static int work_read_count = 0;

volatile u8 user_stop_dma = 0;
volatile u8 dma_finished  = 0;

#define SPI_IMX2_3_CTRL		        0x08
#define SPI_IMX2_3_CTRL_ENABLE		(1 <<  0)
#define SPI_IMX2_3_CTRL_XCH		(1 <<  2)
#define SPI_IMX2_3_CTRL_MODE_MASK	(0xf << 4)
#define SPI_IMX2_3_CTRL_POSTDIV_OFFSET	8
#define SPI_IMX2_3_CTRL_PREDIV_OFFSET	12
#define SPI_IMX2_3_CTRL_CS(cs)		((cs) << 18)
#define SPI_IMX2_3_CTRL_BL_OFFSET	20
#define SPI_IMX2_3_CTRL_SMC 	(1<<3)


#define SPI_IMX2_3_CONFIG	0x0c
#define SPI_IMX2_3_CONFIG_SCLKPHA(cs)	(1 << ((cs) +  0))
#define SPI_IMX2_3_CONFIG_SCLKPOL(cs)	(1 << ((cs) +  4))
#define SPI_IMX2_3_CONFIG_SBBCTRL(cs)	(1 << ((cs) +  8))
#define SPI_IMX2_3_CONFIG_SSBPOL(cs)	(1 << ((cs) + 12))
#define SPI_IMX2_3_CONFIG_SCLKCTL(cs)	(1 << ((cs) + 20))

#define SPI_IMX2_3_INT		0x10

#define SPI_IMX2_3_INT_TEEN		(1<<0)
#define SPI_IMX2_3_INT_RREN		(1<<3)
#define SPI_IMX2_3_INT_RDREN            (1<<4)
#define SPI_IMX2_3_INT_TDREN            (1<<1)
#define SPI_IMX2_3_DMA_REG	        0x14
#define SPI_IMX2_3_DMA_TEDEN            (1<<7)
#define SPI_IMX2_3_DMA_RXDEN            (1<<23)
#define SPI_IMX2_3_DMA_RXTDEN           (1<<31)
#define SPI_IMX2_3_DMA_RX_TH_OFFSET      16
#define SPI_IMX2_3_DMA_TX_TH_OFFSET      0
#define SPI_IMX2_3_DMA_RX_DMA_LEN_OFFSET 24


#define SPI_IMX2_3_STAT		0x18

#define SPI_IMX2_3_TEST_REG          0x20
#define SPI_IMX2_3_STAT_RR	    (1<<3)
#define SPI_IMX2_3_STAT_RO          (1<<6)


static unsigned int spi_imx2_3_clkdiv(unsigned int fin, unsigned int fspi)
{
	/*
	 * there are two 4-bit dividers, the pre-divider divides by
	 * $pre, the post-divider by 2^$post
	 */
	unsigned int pre, post;

	if (unlikely(fspi > fin))
		return 0;

	post = fls(fin) - fls(fspi);
	if (fin > fspi << post)
		post++;

	/* now we have: (fin <= fspi << post) with post being minimal */

	post = max(4U, post) - 4;
	if (unlikely(post > 0xf)) {
		pr_err("%s: cannot set clock freq: %u (base freq: %u)\n",
				__func__, fspi, fin);
		return 0xff;
	}

	pre = DIV_ROUND_UP(fin, fspi << post) - 1;

	pr_debug("%s: fin: %u, fspi: %u, post: %u, pre: %u\n",
			__func__, fin, fspi, post, pre);
	return (pre << SPI_IMX2_3_CTRL_PREDIV_OFFSET) |
		(post << SPI_IMX2_3_CTRL_POSTDIV_OFFSET);
}


static void __maybe_unused spi_imx2_3_slave_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned val = 0;

    //printk("get into function spi_imx2_3_slave_intctrl !!!\n");
	if (enable & MXC_INT_TE)
		val |= SPI_IMX2_3_INT_TEEN;

	if (enable & MXC_INT_RR)
		val |= SPI_IMX2_3_INT_RREN;
	
	if(enable&MXC_INT_TDR)
		val|=SPI_IMX2_3_INT_TDREN;
        //printk("spi_imx2_3_slave_intctrl:val = %d \n",val);

	writel(val, spi_imx->membase + SPI_IMX2_3_INT);
	//printk("get into function spi_imx2_3_slave_intctrl!!!\n");
}

static void __maybe_unused spi_imx2_3_slave_trigger(struct spi_imx_data *spi_imx)
{
	u32 dma=0;		
	

    dma=SPI_IMX2_3_DMA_RXDEN|(31<<SPI_IMX2_3_DMA_RX_TH_OFFSET);	        
	writel(dma, spi_imx->membase + SPI_IMX2_3_DMA_REG);
    //printk("get into function spi_imx2_3_slave_triggerl !!!\n");
		
	return;
}

static void __maybe_unused spi_imx2_3_slave_disable_dma(struct spi_imx_data *spi_imx)
{
	u32 dma=0;		
	
        dma= 0 << 23;	        
	writel(dma, spi_imx->membase + SPI_IMX2_3_DMA_REG);
        printk("disable spi dma ********************************************************************************************************************************!!!\n");
	return;
}


static int __maybe_unused spi_imx2_3_slave_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{

	u32 ctrl = SPI_IMX2_3_CTRL_ENABLE, cfg = 0;

	writel(0, spi_imx->membase + SPI_IMX2_3_CTRL);  //reset control register
 
	ctrl |=SPI_IMX2_3_CTRL_MODE_MASK^(1<<(config->cs+4));
        ctrl |= SPI_IMX2_3_CTRL_CS(config->cs);
	
	/* set clock speed */
       									   
	ctrl |= spi_imx2_3_clkdiv(spi_imx->spi_clk, config->speed_hz);

	ctrl |= (config->bpw - 1) << SPI_IMX2_3_CTRL_BL_OFFSET;


	cfg &=~(0xf<<8);  /*SSB_CTL need clear*/

	if (config->mode & SPI_CPHA)
		cfg |= SPI_IMX2_3_CONFIG_SCLKPHA(config->cs);

	if (config->mode & SPI_CPOL)
		cfg |= SPI_IMX2_3_CONFIG_SCLKPOL(config->cs);
  
        cfg |= SPI_IMX2_3_CONFIG_SSBPOL(config->cs);

	writel(ctrl, spi_imx->membase + SPI_IMX2_3_CTRL);
	writel(cfg, spi_imx->membase + SPI_IMX2_3_CONFIG);
	return 0;
}

static int __maybe_unused spi_imx2_3_slave_rx_available(struct spi_imx_data *spi_imx)
{
	//printk("get into function spi_imx2_3_slave_rx_available !!!\n");
    int rc=readl(spi_imx->membase + SPI_IMX2_3_STAT) & SPI_IMX2_3_STAT_RR;
	return rc;
}

static void  __maybe_unused spi_imx2_3_slave_reset(struct spi_imx_data *spi_imx)
{
	//printk("get into function spi_imx2_3_slave_reset !!!\n");
    /* drain receive buffer */
       
#if 0    
	while (spi_imx2_3_slave_rx_available(spi_imx))
		readl(spi_imx->membase + MXC_CSPIRXDATA);
#endif	
	writel(0, spi_imx->membase + SPI_IMX2_3_CTRL); 
}


/*
 * These version numbers are taken from the Freescale driver.  Unfortunately it
 * doesn't support i.MX1, so this entry doesn't match the scheme. :-(
 */
static struct spi_imx_devtype_data spi_imx_devtype_data[] __devinitdata = {


	[SPI_IMX_VER_2_3_SLAVE] = {
		.intctrl = spi_imx2_3_slave_intctrl,
		.config  = spi_imx2_3_slave_config,
		.trigger = spi_imx2_3_slave_trigger,
		.rx_available = spi_imx2_3_slave_rx_available,
		.reset   = spi_imx2_3_slave_reset,
		.fifosize = 64,
		},

};


static int spi_imx_dma_init(struct spi_imx_data	*spi_imx);
 
static int spidev_rxdma_start(struct spi_imx_data *spi_imx)
{

	struct dma_async_tx_descriptor *desc;
	struct scatterlist * sg=NULL;
	struct dma_chan *chan;
	unsigned long start_flag;


	spidev_data_frame_t *frame;

	chan = spi_imx->dma_chan_rx;
	
	frame = list_entry(spi_imx->working_q.next, spidev_data_frame_t, queue);

	sg = &frame->rx_sgl;

	sg->length = 60*1024;
	
	
    spin_lock_irqsave(&spi_imx->lock, start_flag);
	
	
	desc=chan->device->device_prep_slave_sg(chan,sg,1,DMA_DEV_TO_MEM,0);

	if(!desc){
		printk("spidev_rxdma_start---> Can not init dma descriptor111 \n");
		spin_unlock_irqrestore(&spi_imx->lock, start_flag);
		return -1;
	}

#if 1
    work_read_count++;
#endif    
	
	spin_unlock_irqrestore(&spi_imx->lock, start_flag);
		
	desc->callback=spi_imx_dma_rx_callback;

	desc->callback_param=spi_imx;
	
	spin_lock_irqsave(&spi_imx->lock, start_flag);

	dmaengine_submit(desc);

	//dma_async_issue_pending(spi_imx->dma_chan_rx);
	
	spin_unlock_irqrestore(&spi_imx->lock, start_flag);

    //printk("go out spidev_rxdma_start---> funciton !!! \n");
	return 0;
   
}

static void spidev_free_mem_rxframe(struct spi_imx_data *spi_imx)
{
    int i = 0;
    u32 *ptr = NULL;
    struct scatterlist *sg;


    for (i = 0; i < DMA_BUF_SZIE; i ++) {

    	ptr = spi_imx->rxsegnums[i].vaddress;
            
        sg = &spi_imx->rxsegnums[i].rx_sgl;
        if (ptr != NULL){
                 
            dma_unmap_sg(spi_imx->dev, sg, 1, DMA_FROM_DEVICE);

            //printk("free %d rxframe.\n", i);

            kfree(spi_imx->rxsegnums[i].vaddress);

            spi_imx->rxsegnums[i].vaddress = NULL;
        }
    }
    return;
}


static u32 rec_finsh_count = 0;

 

static imx6q_spi_slave_release_fasync(int fd,struct file *filp,int mode)
{

    struct spi_imx_data *dev = filp->private_data;

    fasync_helper(fd,filp,mode,&dev->fasync_queue);
}  

static int imx6q_spi_slave_fasync(int fd, struct file * filp, int on) 
{
    int retval;  
    struct spi_imx_data *dev = filp->private_data;


    retval=fasync_helper(fd,filp,on,&dev->fasync_queue);  
    if(retval<0)
    {
        return retval;
    }
    return 0;
}

//u32 cmpdata[1024] = { 0} ;
//u32 rectestdata[15*1024] ={0};
  	
#if 1  //patch spi-slave-dma //gxl 2016.5.31
static void spi_imx_dma_rx_callback(void * data)
{
	struct spi_imx_data *spi_imx=data;
	u32 val = 0;
	//u32 i,ret ;
	spidev_data_frame_t *done_frame;
	//struct scatterlist *sg;
	unsigned long flag;
	//u32 *p;

	
	//spin_lock_irqsave(&spi_imx->lock, flag);

	val = readl(spi_imx->membase + SPI_IMX2_3_STAT);

	if(val & SPI_IMX2_3_STAT_RO){
	
		printk("rx FIFO overflow!! ,rec_finsh_count = %d\n",rec_finsh_count);
	 
	}
#if 1	
    if(user_stop_dma){
		
		dma_finished = 1;
		spi_imx->devtype_data.intctrl(spi_imx, 0);
		spi_imx2_3_slave_disable_dma(spi_imx);
		
		return;
	}
#endif	
	done_frame = list_entry(spi_imx->working_q.next, spidev_data_frame_t,queue);	
			 
	//sg = &done_frame->rx_sgl;

	//dma_sync_sg_for_cpu(spi_imx->dev, sg, 1, DMA_FROM_DEVICE);

	rec_finsh_count++;	  

	//printk("spi_imx_dma_rx_callback function  -->step:1!!\n");

	list_del(spi_imx->working_q.next);

	//printk("spi_imx_dma_rx_callback function  -->step:2!!\n");
	
	
	list_add_tail(&done_frame->queue, &spi_imx->ready_q);

	//printk("spi_imx_dma_rx_callback function  -->step:3!!\n");

	if (list_empty(&spi_imx->working_q)){

		printk("write list working_q empty 222 !!\n");  /*add text*/
		  
		spi_imx->work_queue_state = 1;

		//spin_unlock_irqrestore(&spi_imx->lock, flag);

		return ;
	}     
	
	//spin_unlock_irqrestore(&spi_imx->lock, flag);
	
	spidev_rxdma_start(spi_imx);

	//printk("spi_imx_dma_rx_callback function lauch out signal  -->step:1!!\n"); 

	if(spi_imx->fasync_queue)
		kill_fasync(&spi_imx->fasync_queue,SIGIO,POLL_IN);

	//printk("spi_imx_dma_rx_callback function lauch out signal  -->step:2!!\n"); 
         
}

static bool imx_spi_filter(struct dma_chan *chan,void *param)
{
	if (!imx_dma_is_general_purpose(chan)){
		return false;
	}
	chan->private= param;
	return true;	
}


static void spi_imx_dma_uninit(struct spi_imx_data *spi_imx)
{
	if(spi_imx->dma_chan_rx){
		dma_release_channel(spi_imx->dma_chan_rx);
		spi_imx->dma_chan_rx=NULL;
	}
}

static int spi_imx_dma_init(struct spi_imx_data	*spi_imx)
{
	dma_cap_mask_t mask;
	struct imx_dma_data	dma_data;
    struct dma_slave_config slave_config;
	int ret=0;
	unsigned long flag;
    struct dma_chan *chan;

	spin_lock_irqsave(&spi_imx->lock, flag);
	
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	spin_unlock_irqrestore(&spi_imx->lock, flag);
	
	dma_data.priority=DMA_PRIO_HIGH;	
	dma_data.dma_request=spi_imx->dma_req_rx;
    dma_data.peripheral_type =IMX_DMATYPE_CSPI;
	
	spin_lock_irqsave(&spi_imx->lock, flag);
	spi_imx->dma_chan_rx=dma_request_channel(mask,imx_spi_filter,&dma_data);
	if(!spi_imx->dma_chan_rx){
		pr_err("request spi rx dma chan failed");
		goto err;
	}
    spin_unlock_irqrestore(&spi_imx->lock, flag);   
	chan = spi_imx->dma_chan_rx;

	slave_config.direction=DMA_DEV_TO_MEM;
	slave_config.src_addr=spi_imx->mapbase + MXC_CSPIRXDATA;
	slave_config.src_addr_width =4;	
	slave_config.src_maxburst = 32*4;   //(rx_threshold +1) * src_addr_width   /*FIXME*/

    spin_lock_irqsave(&spi_imx->lock, flag);
	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret) {
		printk("spi_imx_dma_setup: dmaengine_slave_config failed ret = %d \n",ret);
		spin_unlock_irqrestore(&spi_imx->lock, flag);
		return -EINVAL;
	}	
	spin_unlock_irqrestore(&spi_imx->lock, flag);

	return ret;
err:
	ret=-EINVAL;
	spi_imx_dma_uninit(spi_imx);
	spin_unlock_irqrestore(&spi_imx->lock, flag);
	
	return ret;
}

#endif

static struct platform_device_id spi_imx_devtype[] = { 
    {
		.name = "imx6q-ecspi-slave",
		.driver_data = SPI_IMX_VER_2_3_SLAVE,
	}, 
     
};

static int spidev_alloc_mem_rxframe(struct spi_imx_data *spi_devdata )
{
	int i = 0;
	struct scatterlist *sg;
	int ret;


	if (!spi_devdata)
		return -1;

	spi_devdata->rxdma_buf_size = 60*1024;
	spi_devdata->num_rxdma_bufs = DMA_BUF_SZIE;

	for (i = 0; i < spi_devdata->num_rxdma_bufs; i++) {

		spi_devdata->rxsegnums[i].vaddress = kzalloc(spi_devdata->rxdma_buf_size, GFP_KERNEL | GFP_DMA);
		if (!spi_devdata->rxsegnums[i].vaddress) {
			printk("Can not allocate vaddress.\n");
			return -1;
		}

        //printk("spidev_alloc_mem_rxframe: ---->one \n"); 
             
		sg = &spi_devdata->rxsegnums[i].rx_sgl;

		sg_init_one(sg,spi_devdata->rxsegnums[i].vaddress,spi_devdata->rxdma_buf_size);

		//printk("spidev_alloc_mem_rxframe: ---->two \n"); 

		ret = dma_map_sg(spi_devdata->dev,sg,1,DMA_FROM_DEVICE);

		if (ret == 0) {
			printk("DMA mapping error for RX.\n");
			return -EINVAL;
		}
		spi_devdata->rxsegnums[i].paddress = sg->dma_address;
		printk("paddress = 0x%x ,vaddress = 0x%x \n",spi_devdata->rxsegnums[i].paddress,spi_devdata->rxsegnums[i].vaddress);

	}

    spi_devdata->work_queue_state  = 0;
    spi_devdata->ready_queue_state = 0;

	printk("spidev_alloc_mem_rxframe: step--->2 \n"); 
	return 0;
}

static void spidev_reset_mem_rxframe(struct spi_imx_data *spi_devdata)
{

     int i = 0;

     
     for (i = 0; i < DMA_BUF_SZIE; i++){

        memset(spi_devdata->rxsegnums[i].vaddress,0,60*1024);
     }
     return;

}

static int spidev_init_rxsegnums_buf(struct spi_imx_data *spi_devdata) {
	
	int i = 0;

	//unsigned long flag;

	if (!spi_devdata)
		return -1;
      
	for (i = 0; i < DMA_BUF_SZIE; i++) {
		//spi_devdata->rxsegnums[i].buffer.offset = spi_devdata->rxsegnums[i].paddress;
		//spi_devdata->rxsegnums[i].buffer.index = i;
		//spi_devdata->rxsegnums[i].buffer.flags = 0x01;         //SPIDEV_BUF_FLAG_MAPPED
		//spi_devdata->rxsegnums[i].buffer.length = 60*1024;
		//spi_devdata->rxsegnums[i].index = i;
                
        //printk("go to spidev_init_rxsegnums_buf --->one ,%d\n",i);

		list_add_tail(&spi_devdata->rxsegnums[i].queue, &spi_devdata->working_q);

        //printk("go to spidev_init_rxsegnums_buf ---->two,%d \n",i);                 

		//spi_devdata->rxsegnums[i].buffer.flags |= 0x02;       //SPIDEV_BUF_FLAG_QUEUED
	}
	//printk("spidev_init_rxsegnums_buf: step--->3 \n");

    return 0;
}

static int dma_enable_count = 0;

static int spidev_rxstreamon(struct spi_imx_data *spi_imx)
{ 
	struct dma_async_tx_descriptor *desc;
	int dir=DMA_TO_DEVICE;
    int ret_init_value = 0;
	struct scatterlist * sg=NULL;      	
	unsigned long flag;
	spidev_data_frame_t *frame;
	struct spi_imx_config config;

	
	if(dma_enable_count < 1)
	{
	    dma_enable_count++;
		ret_init_value = spi_imx_dma_init(spi_imx);
        if(ret_init_value <0)
        {
           printk("dma initial fail: step--->0 \n");
	       return -1;
	    }
	}
	config.bpw = 32;
	config.speed_hz  = 6500000;
	config.mode = 0;
	config.cs = 0;
	spi_imx->bpw = 32;               
	spi_imx->rx_threshold = 31;
	  		
    printk("dma enable: step--->1 \n");
	//spin_lock_irqsave(&spi_imx->lock, flag);
	spi_imx->devtype_data.config(spi_imx, &config);
	//spin_unlock_irqrestore(&spi_imx->lock, flag);

	printk("dma enable: step--->2 \n"); 						
				
	mdelay(50); 
#if 1
	spidev_deinit_rxframe_buf(spi_imx);
	spidev_init_rxsegnums_buf(spi_imx);
#endif

	printk("dma enable: step--->3 \n");	
    spin_lock_irqsave(&spi_imx->lock, flag); 
	
	frame = list_entry(spi_imx->working_q.next, spidev_data_frame_t, queue);
    spin_unlock_irqrestore(&spi_imx->lock, flag);
	printk("dma enable: step--->4 \n");
        
	sg = &frame->rx_sgl;
    printk("paddress = 0x%x ,vaddress = 0x%x \n",frame->paddress,frame->vaddress);
	
    sg->length = 60*1024;
		
    spin_lock_irqsave(&spi_imx->lock, flag); 
	desc=spi_imx->dma_chan_rx->device->device_prep_slave_sg(spi_imx->dma_chan_rx,sg,1,DMA_DEV_TO_MEM,0);
        
	if(!desc){
		printk("spidev_rxstreamon---> Can not init dma descriptor222 \n");  
        spin_unlock_irqrestore(&spi_imx->lock, flag);
        mdelay(100);		
		return -1;
	}
    spin_unlock_irqrestore(&spi_imx->lock, flag);
	
    printk("dma enable: step--->5 \n");

	desc->callback=spi_imx_dma_rx_callback;
        
	desc->callback_param= spi_imx;

	spin_lock_irqsave(&spi_imx->lock, flag); 
	
	dmaengine_submit(desc);

	//dma_async_issue_pending(spi_imx->dma_chan_rx);
	     
	/* enable rx/tx DMA transfer */
	spi_imx->devtype_data.trigger(spi_imx);
	
	spin_unlock_irqrestore(&spi_imx->lock, flag);
  
	printk("dma enable: step--->6 \n");

    return 0;     
}

static void spidev_imx_rxdma_exit(struct spi_imx_data* spi_devdata)
{
	if (spi_devdata->dma_chan_rx) {
		dma_release_channel(spi_devdata->dma_chan_rx);
		spi_devdata->dma_chan_rx = NULL;
	}
}

static void spidev_deinit_rxframe_buf(struct spi_imx_data  *spi_devdata)
{
	INIT_LIST_HEAD(&spi_devdata->ready_q);
	//INIT_LIST_HEAD(&spi_devdata->done_q);
	INIT_LIST_HEAD(&spi_devdata->working_q);
}


static int spidev_rxstreamoff(struct spi_imx_data *spi_devdata)
{
	unsigned long flag;
	
#if 1
    user_stop_dma = 1;
    while(dma_finished == 0);
#endif
     	
	if (!spi_devdata)
	    return -EIO;
        
    //spi_imx2_3_slave_disable_dma(spi_devdata);

	//spidev_imx_rxdma_exit(spi_devdata); 

	printk("work_read_count %d \n" ,work_read_count);  
	//spidev_deinit_rxframe_buf(spi_devdata);
    //spidev_reset_mem_rxframe(spi_devdata);

    /* free rxframe dma mem. */
	//spidev_free_mem_rxframe(spi_devdata);
	return 0;
}

static int spidev_imx_open(struct inode *inode, struct file *filp)
{
	struct spi_imx_data *spi_devdata;
	int ret = -ENXIO;
	unsigned long lock_flags;
    //u16 reg_statue;
    //u32 reg_dma;
    //u32 reg_spiconfig;
    //u32 reg_ctl;


	mutex_lock(&device_list_lock);

	list_for_each_entry(spi_devdata, &device_list, device_entry) {
		if (spi_devdata->devt == inode->i_rdev) {
			ret = 0;
			break;
		}
	}

	if (ret != 0) {
	  mutex_unlock(&device_list_lock);
	  return -ENODEV;
	}

    filp->private_data = spi_devdata;
    nonseekable_open(inode, filp);
    /* enable the clk of eCSPI */
    clk_enable(spi_devdata->clk);

    //spidev_rxstreamon(spi_devdata);     
				   
    mutex_unlock(&device_list_lock);      
   
    //spin_lock_irqsave(&spi_devdata->rx_int_lock, lock_flags);
    //spi_devdata->rx_counter = 0;
    //spin_unlock_irqrestore(&spi_devdata->rx_int_lock, lock_flags);
#if 0
    	
	INIT_LIST_HEAD(&spi_devdata->ready_q);
    //INIT_LIST_HEAD(&spi_devdata->done_q);
    INIT_LIST_HEAD(&spi_devdata->working_q);
#endif	
    return ret;
}

static ssize_t spidev_imx_read(struct file *filp, char __user *buf,
    size_t count, loff_t *f_pos)
{
	spidev_data_frame_t *ready_frame;
    struct spi_imx_data *spi_devdata;
	spi_devdata = filp->private_data;
        
       
    //printk("get into function ---->spidev_imx_read !!!\n");
	if (list_empty(&spi_devdata->ready_q)){

		printk("list ready_q empty !!\n");        /*add text*/

		spi_devdata->ready_queue_state = 1;
		return -1;
	}        
	if( buf == NULL)
	{

		printk("user read buf is null pointer !!\n");
		return -2;
	}         
	ready_frame = list_entry(spi_devdata->ready_q.next, spidev_data_frame_t, queue);
	
	copy_to_user(buf, ready_frame->vaddress, 60*1024);

	list_del(spi_devdata->ready_q.next);        

	list_add_tail(&ready_frame->queue, &spi_devdata->working_q);
    
    work_read_count--;
 
    return 0;
}

static long spidev_imx_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct spi_imx_data *spi_devdata;
    //struct device *dev;
    int ret = 0;

    spi_devdata = filp->private_data;
    if (!spi_devdata)
    return -EBADF;

    //if (down_interruptible(&spi_devdata->busy_lock))
    //		return -EINTR;

    switch (cmd) {        
	    case SPIDEV_IOC_SET_RX_DMASIZE: {
		    //unsigned int dma_size;

		    //if (copy_from_user(&dma_size, (void __user *)arg, sizeof(dma_size))) {
		    //	ret = -EFAULT;
		    //	break;
		    //}

		    //spi_devdata->rx_dma_size = dma_size;
		    break;
	    }
        case SPIDEV_IOC_QUEUE_STATE: {
			static u8  state_flag = 0;
			if(spi_devdata->ready_queue_state){
				state_flag |= 1<<0;
			}else{
				state_flag |= 0<<0;
			}
			if(spi_devdata->work_queue_state){
				state_flag |= 1<<1;
			}else{
				state_flag |= 0<<1;
			}

			copy_to_user((void __user *)arg, &state_flag, 1);
		
		    break;
	    }

	case SPIDEV_IOC_RXSTREAMON: {
		printk("DMA ENABLE======0000====\n"); 

	#if 1	
        user_stop_dma = 0;
		dma_finished = 0;
    #endif
 	
		ret = spidev_rxstreamon(spi_devdata); //使能 
		if( ret == 0)
		{
			printk("DMA ENABLE=====1111=====\n"); 
		}else{
			printk("ret = %d\n\r",ret); 
		}
		break;
	}

	case SPIDEV_IOC_RXSTREAMOFF: {
		ret = spidev_rxstreamoff(spi_devdata);
	break;
	}
	default:
		break;
   }
  //up(&spi_devdata->busy_lock);
  return ret;
}



static int spidev_imx_release(struct inode *inode, struct file *filp)
{
	struct spi_imx_data *spi_devdata;
        
	//unsigned long lock_flags;

	
    printk("close fd --->spidev_imx_release\n");       
    imx6q_spi_slave_release_fasync(-1, filp, 0);
	mutex_lock(&device_list_lock);
	spi_devdata = filp->private_data;
	filp->private_data = NULL;
	spi_devdata->devtype_data.reset(spi_devdata);
	spi_devdata->devtype_data.intctrl(spi_devdata, 0);
	//spidev_imx_rxdma_exit(spi_devdata);
	clk_disable(spi_devdata->clk);
	mutex_unlock(&device_list_lock);
	return 0;
}

static const struct file_operations spidev_imx_fops = {
	.owner = THIS_MODULE,
	.open  = spidev_imx_open,
	.read  = spidev_imx_read,
	.unlocked_ioctl = spidev_imx_unlocked_ioctl,
	.release = spidev_imx_release,
    .fasync  = imx6q_spi_slave_fasync,
    //.mmap  = spidev_imx_mmap,
};

static int __devinit spidev_imx_probe(struct platform_device *pdev)
{
	struct spi_imx_data  *spi_devdata;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct resource *ioarea;
	void __iomem *mem_base;
	struct clk *clk;
	int ret;
	int irq;
	unsigned long minor;

    printk("get into function ---> spidev_imx_probe \n");
	spi_devdata = kzalloc(sizeof(*spi_devdata), GFP_KERNEL);
	if (!spi_devdata) {
		printk("allocate spi_devdata fail.\n");
		return -ENOMEM;
	}

	spin_lock_init(&spi_devdata->spi_lock);
	//spin_lock_init(&spi_devdata->rx_int_lock);
	//sema_init(&spi_devdata->busy_lock, 1);
	INIT_LIST_HEAD(&spi_devdata->device_entry);
	//init_waitqueue_head(&spi_devdata->rx_dma_wait);


	spi_devdata->bus_num = pdev->id;

	spi_devdata->devtype_data = spi_imx_devtype_data[0];
                                    
	/* get clock */
	//clk = clk_get(dev, NULL); 
    clk = clk_get_sys("imx6q-ecspi.1", NULL);
	if (IS_ERR(clk)) {
		dev_err(dev, "unable to get clock\n");
		ret = PTR_ERR(clk);
		goto err_out_devdata;
	}

	spi_devdata->clk = clk;

    printk("spi_devdata->clk = %d  \n",spi_devdata->clk);
        
	/* get reg base */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "can't get platform resource\n");
		ret = -ENOMEM;
		goto err_out_devdata;
	}

	spi_devdata->mapbase = res->start;

	ioarea = request_mem_region(res->start,
                                  resource_size(res),
                                  pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		ret = -EBUSY;
		goto err_out_devdata;
	}

	mem_base = ioremap(res->start, resource_size(res));
	if (!mem_base) {
		dev_err(dev, "Cannot map IO.\n");
		ret = -EINVAL;
		goto out_release_mem;
	}
     
	spi_devdata->membase = mem_base;
 
 
    printk("spi_devdata->membase = %d  \n",spi_devdata->membase);
	/* allocate rxframe buffer */
 	
    ret = spidev_alloc_mem_rxframe( spi_devdata );
    if (ret != 0) {
        printk("error allocate rxframe memory\n");
        goto out_iounmap;
           
    }
	
#if 1
    INIT_LIST_HEAD(&spi_devdata->ready_q);
    //INIT_LIST_HEAD(&spi_devdata->done_q);
    INIT_LIST_HEAD(&spi_devdata->working_q);
    spidev_init_rxsegnums_buf(spi_devdata);	
#endif		
	/* get irq request line */
#ifdef ENABLE_SPI_INTERRUPT

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = -EINVAL;
		goto out_free_mem_rxframe;
	}
	spi_devdata->irq = irq;

	/* request irq */

	ret = request_irq(irq, spidev_imx_isr, 0, DRIVER_NAME, spi_devdata);
	if (ret) {
		dev_err(dev, "can't get irq%d: %d\n", irq, ret);
		goto out_free_mem_rxframe;
	}
#endif
  
#if 0 
	/* get rx dma request line */
	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "rx");
	if (!res) {
		dev_err(dev, "unable to get rx dma\n");
		goto out_free_irq;
	}
	spi_devdata->dma_req_rx = res->start;

	/* get tx dma request line */
	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "tx");
	if (!res) {
		dev_err(dev, "unable to get tx dma\n");
		goto out_free_irq;
	}
	spi_devdata->dma_req_tx = res->start;
#else
    spi_devdata->dma_req_rx = MX6Q_DMA_REQ_CSPI2_RX;
#endif

  	mutex_lock(&device_list_lock);

	minor = find_first_zero_bit(minors, N_SPIDEV_MINORS);
	if (minor < N_SPIDEV_MINORS) {
		spi_devdata->devt = MKDEV(SPIDEV_IMX_MAJOR, minor);

	dev = device_create(spidev_imx_class, dev, spi_devdata->devt,
	    	NULL, "mxc_spidev%d", pdev->id);

	ret = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		printk("no minor number available!\n");
		ret = -ENODEV;
		mutex_unlock(&device_list_lock);
		goto out_free_irq;
	}

	if (ret != 0) {
		mutex_unlock(&device_list_lock);
		goto out_free_irq;
	}

	set_bit(minor, minors);
	list_add(&spi_devdata->device_entry, &device_list);

	mutex_unlock(&device_list_lock);
	ret = clk_enable(clk);
	spi_devdata->spi_clk = clk_get_rate(spi_devdata->clk);
        
    printk("spi_devdata->spi_clk = %d \n",spi_devdata->spi_clk);

    spi_devdata->devtype_data.reset(spi_devdata);
    //spi_devdata->devtype_data.intctrl(spi_devdata, 0);
	clk_disable(clk);
	spin_lock_irq(&spi_devdata->spi_lock);
  	platform_set_drvdata(pdev, spi_devdata);
  	spin_unlock_irq(&spi_devdata->spi_lock);
    printk("exit function ---> spidev_imx_probe \n");    
  	dev_info(dev, "probed\n");

    return ret;

out_free_irq:
#ifdef ENABLE_SPI_INTERRUPT
    free_irq(irq, spi_devdata);
#endif
out_free_mem_rxframe:
    spidev_free_mem_rxframe(spi_devdata);

out_release_mem:
    release_mem_region(res->start, resource_size(res));
out_iounmap:
    iounmap(mem_base);

err_out_devdata:
    kfree(spi_devdata);

    return ret;
}

static int __devexit spidev_imx_remove(struct platform_device *pdev)
{
    struct spi_imx_data  *spi_devdata = platform_get_drvdata(pdev);
    struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

    //clk_disable(spi_devdata->clk);
	clk_put(spi_devdata->clk);
#ifdef ENABLE_SPI_INTERRUPT
	free_irq(spi_devdata->irq, spi_devdata);
#endif
    /* free rxframe dma mem. */
	spidev_free_mem_rxframe(spi_devdata);

    iounmap(spi_devdata->membase);

    release_mem_region(res->start, resource_size(res));

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spi_devdata->spi_lock);
	platform_set_drvdata(pdev, NULL);
	spin_unlock_irq(&spi_devdata->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&spi_devdata->device_entry);
	clear_bit(MINOR(spi_devdata->devt), minors);
	device_destroy(spidev_imx_class, spi_devdata->devt);

	kfree(spi_devdata);
	mutex_unlock(&device_list_lock);

  	return 0;
}

static struct platform_driver spidev_imx_driver = {
	.driver = {
	.name = DRIVER_NAME,
	.owner = THIS_MODULE,
	},
	//.id_table = spi_imx_devtype,
	.probe = spidev_imx_probe,
	.remove = __devexit_p(spidev_imx_remove),
};

static int __init spidev_imx_init(void)
{
	int ret = -ENODEV;

    printk("start register slave_spi_dev --->step1\n");
	ret = register_chrdev(SPIDEV_IMX_MAJOR, "mxc_spidev", &spidev_imx_fops);
	if (ret < 0) {
		printk("register character devicd fail.\n");
		return ret;
	}
    printk("start register slave_spi_dev --->step2\n");
	spidev_imx_class = class_create(THIS_MODULE,"mxc_spidev");
	if (IS_ERR(spidev_imx_class)) {
		printk("class create fail.\n");
		ret = PTR_ERR(spidev_imx_class);
		goto err_out_chrdev;
	}
    printk("start register slave_spi_dev --->step3\n");
	ret = platform_driver_register(&spidev_imx_driver);
	if (ret < 0) {
		goto err_out_class;
	}
        printk("start register slave_spi_dev --->step4\n");
	return ret;

err_out_class:
	class_destroy(spidev_imx_class);
err_out_chrdev:
	unregister_chrdev(SPIDEV_IMX_MAJOR, "mxc_spidev");
  return ret;
}

static void __exit spidev_imx_exit(void)
{
	platform_driver_unregister(&spidev_imx_driver);
	class_destroy(spidev_imx_class);
	unregister_chrdev(SPIDEV_IMX_MAJOR, "mxc_spidev");

	printk("remove spidev drivder module.\n");
}

module_init(spidev_imx_init);
module_exit(spidev_imx_exit);

MODULE_AUTHOR("gxl of yinlian");
MODULE_DESCRIPTION("User mode mxc spi device driver");
MODULE_LICENSE("GPL");









