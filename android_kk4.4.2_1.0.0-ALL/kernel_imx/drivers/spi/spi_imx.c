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

#if 1      //patch spi-dma-slave  gxl 2016.5.31
#include <linux/dma-mapping.h>
#endif

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
#include <linux/spi/spi_bitbang.h>
#include <linux/types.h>

#if 1      //patch spi-dma-slave  gxl 2016.5.31
#include <mach/dma.h>
#endif

#include <mach/spi.h>

#define DRIVER_NAME "spi_imx"

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
#if 1  //patch spi-dma-slave  gxl 2016.5.31
#define MXC_INT_TDR (1 << 2)
#define MXC_INT_RDR (1 << 3)
#define MXC_INT_RF  (1 << 5)
#define SPI_BUF_SIZE PAGE_SIZE	
#endif


//u32  rec_process[256] = {0};
u32  rec_process[1024] = {0};
u32  re_tstdata[1024 *2560] ={0};

u32  backupdata[1024] = {0};




//u8  rec_process[2000] = {0};
//u8 *  re_tstdata ;

static int rec_num=0;
struct spi_imx_config {
	unsigned int speed_hz;
	unsigned int bpw;
	unsigned int mode;
	u8 cs;
};

enum spi_imx_devtype {
#if 0	
        SPI_IMX_VER_IMX1,
	SPI_IMX_VER_0_0,
	SPI_IMX_VER_0_4,
	SPI_IMX_VER_0_5,
	SPI_IMX_VER_0_7,
#endif
	SPI_IMX_VER_2_3,
#if 1    //patch spi-dma-slave  //gxl 2016.5.31
        SPI_IMX_VER_2_3_SLAVE,
#endif
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
  int index;    
  struct spidev_buffer  buffer;
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
	struct spi_bitbang bitbang;
#if 1  //patch spi-dma-slave  //gxl 2016.5.31
       spinlock_t lock;
#endif
	struct completion xfer_done;
	void *base;
	int irq;
	struct clk *clk;
	unsigned long spi_clk;
	int *chipselect;
#if 1  //patch spi-dma-slave  //gxl 2016.5.31
	int count;
#endif
	void (*tx)(struct spi_imx_data *);
	void (*rx)(struct spi_imx_data *);
	void *rx_buf;
	const void *tx_buf;
#if 1  //patch spi-dma-slave  //gxl 2016.5.31
	int txfifo; /* number of words pushed in tx FIFO */
        int master_mode; 			/*slave or master*/
	unsigned int dma_number;
	unsigned int rx_threshold;  
	unsigned int tx_threshold;   	
	unsigned int dma_req_rx;
	unsigned int dma_req_tx;
	unsigned int enable_dma;		
	unsigned int dma_inited;		
	unsigned int dma_or_pio;			
	unsigned int bpw;
	struct dma_chan	*dma_chan_rx;
	struct dma_chan	*dma_chan_tx;
	void   *dma_tmp_buf;
	struct scatterlist	rx_sg;
   	struct scatterlist	tx_sg;
	struct dma_async_tx_descriptor *tx_desc;
	struct dma_async_tx_descriptor *rx_desc;
	struct device *dev;
	resource_size_t mapbase;

        spidev_data_frame_t rxsegnums[320]; 
        struct list_head ready_q; 
  	struct list_head done_q;  
  	struct list_head working_q;
       
        u32 rxdma_buf_size ;
	u32 num_rxdma_bufs ;
#endif
	struct spi_imx_devtype_data devtype_data;
};

static int spidev_rxstreamon(struct spi_imx_data *spi_imx);
static void spi_imx_dma_rx_callback(void * data);

static int spidev_alloc_mem_rxframe(struct spi_imx_data *spi_devdata );
static int spidev_init_rxsegnums_buf(struct spi_imx_data *spi_devdata);
static void spidev_free_mem_rxframe(struct spi_imx_data *spi_imx);

#define MXC_SPI_BUF_RX(type)						\
static void spi_imx_buf_rx_##type(struct spi_imx_data *spi_imx)		\
{									\
	unsigned int val = readl(spi_imx->base + MXC_CSPIRXDATA);	\
									\
	if (spi_imx->rx_buf) {						\
		*(type *)spi_imx->rx_buf = val;				\
		spi_imx->rx_buf += sizeof(type);			\
	}								\
}

#define MXC_SPI_BUF_TX(type)						\
static void spi_imx_buf_tx_##type(struct spi_imx_data *spi_imx)		\
{									\
	type val = 0;							\
									\
	if (spi_imx->tx_buf) {						\
		val = *(type *)spi_imx->tx_buf;				\
		spi_imx->tx_buf += sizeof(type);			\
	}								\
									\
	spi_imx->count -= sizeof(type);					\
									\
	writel(val, spi_imx->base + MXC_CSPITXDATA);			\
}

MXC_SPI_BUF_RX(u8)
MXC_SPI_BUF_TX(u8)
MXC_SPI_BUF_RX(u16)
MXC_SPI_BUF_TX(u16)
MXC_SPI_BUF_RX(u32)
MXC_SPI_BUF_TX(u32)




/* First entry is reserved, second entry is valid only if SDHC_SPIEN is set
 * (which is currently not the case in this driver)
 */
static int mxc_clkdivs[] = {0, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192,
	256, 384, 512, 768, 1024};

/* MX21, MX27 */
static unsigned int spi_imx_clkdiv_1(unsigned int fin,
		unsigned int fspi)
{
	int i, max;

	if (cpu_is_mx21())
		max = 18;
	else
		max = 16;

	for (i = 2; i < max; i++)
		if (fspi * mxc_clkdivs[i] >= fin)
			return i;

	return max;
}

/* MX1, MX31, MX35, MX51 CSPI */
static unsigned int spi_imx_clkdiv_2(unsigned int fin,
		unsigned int fspi)
{
	int i, div = 4;

	for (i = 0; i < 7; i++) {
		if (fspi * div >= fin)
			return i;
		div <<= 1;
	}

	return 7;
}

#define SPI_IMX2_3_CTRL		0x08
#define SPI_IMX2_3_CTRL_ENABLE		(1 <<  0)
#define SPI_IMX2_3_CTRL_XCH		(1 <<  2)
#define SPI_IMX2_3_CTRL_MODE_MASK	(0xf << 4)
#define SPI_IMX2_3_CTRL_POSTDIV_OFFSET	8
#define SPI_IMX2_3_CTRL_PREDIV_OFFSET	12
#define SPI_IMX2_3_CTRL_CS(cs)		((cs) << 18)
#define SPI_IMX2_3_CTRL_BL_OFFSET	20

#if 1   //patch spi-spi-slave-dma //gxl 2016.5.31
#define SPI_IMX2_3_CTRL_SMC 	(1<<3)
#endif

#define SPI_IMX2_3_CONFIG	0x0c
#define SPI_IMX2_3_CONFIG_SCLKPHA(cs)	(1 << ((cs) +  0))
#define SPI_IMX2_3_CONFIG_SCLKPOL(cs)	(1 << ((cs) +  4))
#define SPI_IMX2_3_CONFIG_SBBCTRL(cs)	(1 << ((cs) +  8))
#define SPI_IMX2_3_CONFIG_SSBPOL(cs)	(1 << ((cs) + 12))

#if 1  //patch spi-spi-slave-dma //gxl 2016.5.31
#define SPI_IMX2_3_CONFIG_SCLKCTL(cs)	(1 << ((cs) + 20))
#endif

#define SPI_IMX2_3_INT		0x10

#if 1  //patch spi-spi-slave-dma //gxl 2016.5.31
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
#endif

#define SPI_IMX2_3_STAT		0x18

#if 1  //patch spi-spi-slave-dma //gxl 2016.5.31
#define SPI_IMX2_3_TEST_REG          0x20
#define SPI_IMX2_3_STAT_RR	    (1<<3)
#define SPI_IMX2_3_STAT_RO          (1<<6)
#endif

/* MX51 eCSPI */
#if 1  //patch spi-spi-slave-dma //gxl 2016.5.31
static void dump(struct spi_imx_data* spi_imx)
{
	pr_debug("CTRL0x%x CONFIG0x%x INT0x%x STAT0x%x DMA0x%x TEST0x%x\n",
			readl(spi_imx->base + MXC_CSPICTRL),readl(spi_imx->base + SPI_IMX2_3_CONFIG),
			readl(spi_imx->base + SPI_IMX2_3_INT),readl(spi_imx->base + SPI_IMX2_3_STAT),
			readl(spi_imx->base+SPI_IMX2_3_DMA_REG),readl(spi_imx->base+SPI_IMX2_3_TEST_REG));
}
#endif

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

static void __maybe_unused spi_imx2_3_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned val = 0;

	if (enable & MXC_INT_TE)
		val |= SPI_IMX2_3_INT_TEEN;

	if (enable & MXC_INT_RR)
		val |= SPI_IMX2_3_INT_RREN;

#if 0  //patch spi_slave_dma  //gxl 2016.5.31
        if(enable&MXC_INT_TDR)
		val|=SPI_IMX2_3_INT_TDREN;
#endif
	writel(val, spi_imx->base + SPI_IMX2_3_INT);
}

static void __maybe_unused spi_imx2_3_trigger(struct spi_imx_data *spi_imx)
{

#if 1  //patch spi_slave_dma  //gxl 2016.5.31
        u32 reg,dma=0;
		
	//if(spi_imx->dma_or_pio==0){
		reg = readl(spi_imx->base + SPI_IMX2_3_CTRL);
		reg |= SPI_IMX2_3_CTRL_XCH;
		writel(reg, spi_imx->base + SPI_IMX2_3_CTRL);
	//}
	//else{
	//	reg = readl(spi_imx->base + SPI_IMX2_3_CTRL);
	//	reg |= SPI_IMX2_3_CTRL_SMC;
	//	writel(reg, spi_imx->base + SPI_IMX2_3_CTRL);
		//dma=SPI_IMX2_3_DMA_TEDEN|
		//	SPI_IMX2_3_DMA_RXDEN|		
		//	((spi_imx->rx_threshold)<<SPI_IMX2_3_DMA_RX_TH_OFFSET)|
		//	(spi_imx->tx_threshold<<SPI_IMX2_3_DMA_TX_TH_OFFSET);
       //         dma=SPI_IMX2_3_DMA_TEDEN|
	//		SPI_IMX2_3_DMA_RXDEN|		
	//		(0<<SPI_IMX2_3_DMA_RX_TH_OFFSET)|
	//		(0<<SPI_IMX2_3_DMA_TX_TH_OFFSET);
	//	writel(dma, spi_imx->base + SPI_IMX2_3_DMA_REG);
	//}	
#endif       
}

static int __maybe_unused spi_imx2_3_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	u32 ctrl = SPI_IMX2_3_CTRL_ENABLE, cfg = 0;


	//writel(0, spi_imx->base + SPI_IMX2_3_CTRL);
	/*
	 * The hardware seems to have a race condition when changing modes. The
	 * current assumption is that the selection of the channel arrives
	 * earlier in the hardware than the mode bits when they are written at
	 * the same time.
	 * So set master mode for all channels as we do not support slave mode.
	 */
	ctrl |= SPI_IMX2_3_CTRL_MODE_MASK;

	/* set clock speed */
	ctrl |= spi_imx2_3_clkdiv(spi_imx->spi_clk, config->speed_hz);

	/* set chip select to use */
	ctrl |= SPI_IMX2_3_CTRL_CS(config->cs);

	//ctrl |= (config->bpw - 1) << SPI_IMX2_3_CTRL_BL_OFFSET;
        ctrl |= (0x3F) << SPI_IMX2_3_CTRL_BL_OFFSET; //SPI burst lenth 2 word

	cfg |= SPI_IMX2_3_CONFIG_SBBCTRL(config->cs);

        //printk("spi_imx2_3_config: spi_imx->spi_clk = %d ,config->speed_hz = %d\n",spi_imx->spi_clk,config->speed_hz);
        //printk("spi_imx2_3_config: config->bpw = %d ,config->mode = %d\n",config->bpw,config->mode);

        //printk("spi_imx2_3_config: config->cs = %d \n",config->cs);
	if (config->mode & SPI_CPHA)
		cfg |= SPI_IMX2_3_CONFIG_SCLKPHA(config->cs);

	if (config->mode & SPI_CPOL){
		cfg |= SPI_IMX2_3_CONFIG_SCLKPOL(config->cs);
#if 0   //patch slave-spi-dma //gxl 2016.5.31
                cfg |= SPI_IMX2_3_CONFIG_SCLKCTL(config->cs);
#endif
        }
	if (config->mode & SPI_CS_HIGH)
		cfg |= SPI_IMX2_3_CONFIG_SSBPOL(config->cs);

        
	writel(ctrl, spi_imx->base + SPI_IMX2_3_CTRL);
	writel(cfg, spi_imx->base + SPI_IMX2_3_CONFIG);

	return 0;
}

static int __maybe_unused spi_imx2_3_rx_available(struct spi_imx_data *spi_imx)
{
	//return readl(spi_imx->base + SPI_IMX2_3_STAT) & SPI_IMX2_3_STAT_RR;
#if 1   //patch spi-slave-dma //gxl 2016.5.31
        //printk("find  NULL pointe step--->10 !!\n");
        int rc=readl(spi_imx->base + SPI_IMX2_3_STAT) & SPI_IMX2_3_STAT_RR;
	return rc;
#endif

}

static void __maybe_unused spi_imx2_3_reset(struct spi_imx_data *spi_imx)
{
	/* drain receive buffer */

        //printk("find  NULL pointe step--->9 !!\n");
	while (spi_imx2_3_rx_available(spi_imx))
		readl(spi_imx->base + MXC_CSPIRXDATA);
}
#if 1  ////patch spi-slave-dma //gxl 2016.5.31
static void __maybe_unused spi_imx2_3_slave_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned val = 0;

        printk("get into function spi_imx2_3_slave_intctrl !!!\n");
	if (enable & MXC_INT_TE)
		val |= SPI_IMX2_3_INT_TEEN;

	if (enable & MXC_INT_RR)
		val |= SPI_IMX2_3_INT_RREN;
	
	if(enable&MXC_INT_TDR)
		val|=SPI_IMX2_3_INT_TDREN;
        printk("spi_imx2_3_slave_intctrl:val = %d \n",val);

	writel(val, spi_imx->base + SPI_IMX2_3_INT);
}

static void __maybe_unused spi_imx2_3_slave_trigger(struct spi_imx_data *spi_imx)
{
	u32 dma=0;		
	//if(spi_imx->dma_or_pio==0){
	//}
	//else{
		//dma=SPI_IMX2_3_DMA_TEDEN|
		//	SPI_IMX2_3_DMA_RXDEN|
		//	(spi_imx->rx_threshold<<SPI_IMX2_3_DMA_RX_TH_OFFSET)|	
		//	(spi_imx->tx_threshold<<SPI_IMX2_3_DMA_TX_TH_OFFSET);

                dma=SPI_IMX2_3_DMA_RXDEN|
			(31<<SPI_IMX2_3_DMA_RX_TH_OFFSET);	

                //dma=SPI_IMX2_3_DMA_RXDEN|
		//	(2<<SPI_IMX2_3_DMA_RX_TH_OFFSET) | (1<<31) |(3<<24);
                
                
		writel(dma, spi_imx->base + SPI_IMX2_3_DMA_REG);
                //printk("get into function spi_imx2_3_slave_triggerl !!!\n");
	//}	
	return;
}

static int __maybe_unused spi_imx2_3_slave_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{

	u32 ctrl = SPI_IMX2_3_CTRL_ENABLE, cfg = 0;

	writel(0, spi_imx->base + SPI_IMX2_3_CTRL);  //reset control register
 
      #if 1
	ctrl |=SPI_IMX2_3_CTRL_MODE_MASK^(1<<(config->cs+4));
        ctrl |= SPI_IMX2_3_CTRL_CS(config->cs);
      #else  
        ctrl |=SPI_IMX2_3_CTRL_MODE_MASK^(1<<(1+4));
        ctrl |= SPI_IMX2_3_CTRL_CS(1);
      #endif      
	
	/* set clock speed */
       									   
	ctrl |= spi_imx2_3_clkdiv(spi_imx->spi_clk, config->speed_hz);

	ctrl |= (config->bpw - 1) << SPI_IMX2_3_CTRL_BL_OFFSET;


	cfg &=~(0xf<<8);  /*SSB_CTL need clear*/

        //printk("spi_imx2_3_slave_config: spi_imx->spi_clk = %d ,config->speed_hz = %d\n",spi_imx->spi_clk,config->speed_hz);
        //printk("spi_imx2_3_slave_config: config->bpw = %d ,config->mode = %d\n",config->bpw,config->mode);
	if (config->mode & SPI_CPHA)
		cfg |= SPI_IMX2_3_CONFIG_SCLKPHA(config->cs);

	if (config->mode & SPI_CPOL)
		cfg |= SPI_IMX2_3_CONFIG_SCLKPOL(config->cs);

	//if (config->mode & SPI_CS_HIGH)
	//	cfg |= SPI_IMX2_3_CONFIG_SSBPOL(config->cs); 

#if 1   
        cfg |= SPI_IMX2_3_CONFIG_SSBPOL(config->cs);
#else
        
        //cfg |= SPI_IMX2_3_CONFIG_SSBPOL(config->cs);
        //cfg |= SPI_IMX2_3_CONFIG_SBBCTRL(config->cs);
#endif
	writel(ctrl, spi_imx->base + SPI_IMX2_3_CTRL);
	writel(cfg, spi_imx->base + SPI_IMX2_3_CONFIG);
        //printk("get into function spi_imx2_3_slave_config !!!\n");
	return 0;
}

static int __maybe_unused spi_imx2_3_slave_rx_available(struct spi_imx_data *spi_imx)
{
	//printk("get into function spi_imx2_3_slave_rx_available !!!\n");
    int rc=readl(spi_imx->base + SPI_IMX2_3_STAT) & SPI_IMX2_3_STAT_RR;
	return rc;
}

static void  __maybe_unused spi_imx2_3_slave_reset(struct spi_imx_data *spi_imx)
{
	//printk("get into function spi_imx2_3_slave_reset !!!\n");
        /* drain receive buffer */
	while (spi_imx2_3_slave_rx_available(spi_imx))
		readl(spi_imx->base + MXC_CSPIRXDATA);
}

#endif



#define MX31_INTREG_TEEN	(1 << 0)
#define MX31_INTREG_RREN	(1 << 3)

#define MX31_CSPICTRL_ENABLE	(1 << 0)
#define MX31_CSPICTRL_MASTER	(1 << 1)
#define MX31_CSPICTRL_XCH	(1 << 2)
#define MX31_CSPICTRL_POL	(1 << 4)
#define MX31_CSPICTRL_PHA	(1 << 5)
#define MX31_CSPICTRL_SSCTL	(1 << 6)
#define MX31_CSPICTRL_SSPOL	(1 << 7)
#define MX31_CSPICTRL_BC_SHIFT	8
#define MX35_CSPICTRL_BL_SHIFT	20
#define MX31_CSPICTRL_CS_SHIFT	24
#define MX35_CSPICTRL_CS_SHIFT	12
#define MX31_CSPICTRL_DR_SHIFT	16

#define MX31_CSPISTATUS		0x14
#define MX31_STATUS_RR		(1 << 3)

/* These functions also work for the i.MX35, but be aware that
 * the i.MX35 has a slightly different register layout for bits
 * we do not use here.
 */
static void __maybe_unused mx31_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned int val = 0;

	if (enable & MXC_INT_TE)
		val |= MX31_INTREG_TEEN;
	if (enable & MXC_INT_RR)
		val |= MX31_INTREG_RREN;

	writel(val, spi_imx->base + MXC_CSPIINT);
}

static void __maybe_unused mx31_trigger(struct spi_imx_data *spi_imx)
{
	unsigned int reg;

	reg = readl(spi_imx->base + MXC_CSPICTRL);
	reg |= MX31_CSPICTRL_XCH;
	writel(reg, spi_imx->base + MXC_CSPICTRL);
}

static int __maybe_unused spi_imx0_4_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	unsigned int reg = MX31_CSPICTRL_ENABLE | MX31_CSPICTRL_MASTER;
	int cs = spi_imx->chipselect[config->cs];

	reg |= spi_imx_clkdiv_2(spi_imx->spi_clk, config->speed_hz) <<
		MX31_CSPICTRL_DR_SHIFT;

	reg |= (config->bpw - 1) << MX31_CSPICTRL_BC_SHIFT;

	if (config->mode & SPI_CPHA)
		reg |= MX31_CSPICTRL_PHA;
	if (config->mode & SPI_CPOL)
		reg |= MX31_CSPICTRL_POL;
	if (config->mode & SPI_CS_HIGH)
		reg |= MX31_CSPICTRL_SSPOL;
	if (cs < 0)
		reg |= (cs + 32) << MX31_CSPICTRL_CS_SHIFT;

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int __maybe_unused spi_imx0_7_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	unsigned int reg = MX31_CSPICTRL_ENABLE | MX31_CSPICTRL_MASTER;
	int cs = spi_imx->chipselect[config->cs];

	reg |= spi_imx_clkdiv_2(spi_imx->spi_clk, config->speed_hz) <<
		MX31_CSPICTRL_DR_SHIFT;

	reg |= (config->bpw - 1) << MX35_CSPICTRL_BL_SHIFT;
	reg |= MX31_CSPICTRL_SSCTL;

	if (config->mode & SPI_CPHA)
		reg |= MX31_CSPICTRL_PHA;
	if (config->mode & SPI_CPOL)
		reg |= MX31_CSPICTRL_POL;
	if (config->mode & SPI_CS_HIGH)
		reg |= MX31_CSPICTRL_SSPOL;
	if (cs < 0)
		reg |= (cs + 32) << MX35_CSPICTRL_CS_SHIFT;

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int __maybe_unused mx31_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MX31_CSPISTATUS) & MX31_STATUS_RR;
}

static void __maybe_unused spi_imx0_4_reset(struct spi_imx_data *spi_imx)
{
	/* drain receive buffer */
	while (readl(spi_imx->base + MX3_CSPISTAT) & MX3_CSPISTAT_RR)
		readl(spi_imx->base + MXC_CSPIRXDATA);
}

#define MX27_INTREG_RR		(1 << 4)
#define MX27_INTREG_TEEN	(1 << 9)
#define MX27_INTREG_RREN	(1 << 13)

#define MX27_CSPICTRL_POL	(1 << 5)
#define MX27_CSPICTRL_PHA	(1 << 6)
#define MX27_CSPICTRL_SSPOL	(1 << 8)
#define MX27_CSPICTRL_XCH	(1 << 9)
#define MX27_CSPICTRL_ENABLE	(1 << 10)
#define MX27_CSPICTRL_MASTER	(1 << 11)
#define MX27_CSPICTRL_DR_SHIFT	14
#define MX27_CSPICTRL_CS_SHIFT	19

static void __maybe_unused mx27_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned int val = 0;

	if (enable & MXC_INT_TE)
		val |= MX27_INTREG_TEEN;
	if (enable & MXC_INT_RR)
		val |= MX27_INTREG_RREN;

	writel(val, spi_imx->base + MXC_CSPIINT);
}

static void __maybe_unused mx27_trigger(struct spi_imx_data *spi_imx)
{
	unsigned int reg;

	reg = readl(spi_imx->base + MXC_CSPICTRL);
	reg |= MX27_CSPICTRL_XCH;
	writel(reg, spi_imx->base + MXC_CSPICTRL);
}

static int __maybe_unused mx27_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	unsigned int reg = MX27_CSPICTRL_ENABLE | MX27_CSPICTRL_MASTER;
	int cs = spi_imx->chipselect[config->cs];

	reg |= spi_imx_clkdiv_1(spi_imx->spi_clk, config->speed_hz) <<
		MX27_CSPICTRL_DR_SHIFT;
	reg |= config->bpw - 1;

	if (config->mode & SPI_CPHA)
		reg |= MX27_CSPICTRL_PHA;
	if (config->mode & SPI_CPOL)
		reg |= MX27_CSPICTRL_POL;
	if (config->mode & SPI_CS_HIGH)
		reg |= MX27_CSPICTRL_SSPOL;
	if (cs < 0)
		reg |= (cs + 32) << MX27_CSPICTRL_CS_SHIFT;

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int __maybe_unused mx27_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MXC_CSPIINT) & MX27_INTREG_RR;
}

static void __maybe_unused spi_imx0_0_reset(struct spi_imx_data *spi_imx)
{
	writel(1, spi_imx->base + MXC_RESET);
}

#define MX1_INTREG_RR		(1 << 3)
#define MX1_INTREG_TEEN		(1 << 8)
#define MX1_INTREG_RREN		(1 << 11)

#define MX1_CSPICTRL_POL	(1 << 4)
#define MX1_CSPICTRL_PHA	(1 << 5)
#define MX1_CSPICTRL_XCH	(1 << 8)
#define MX1_CSPICTRL_ENABLE	(1 << 9)
#define MX1_CSPICTRL_MASTER	(1 << 10)
#define MX1_CSPICTRL_DR_SHIFT	13

static void __maybe_unused mx1_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned int val = 0;

	if (enable & MXC_INT_TE)
		val |= MX1_INTREG_TEEN;
	if (enable & MXC_INT_RR)
		val |= MX1_INTREG_RREN;

	writel(val, spi_imx->base + MXC_CSPIINT);
}

static void __maybe_unused mx1_trigger(struct spi_imx_data *spi_imx)
{
	unsigned int reg;

	reg = readl(spi_imx->base + MXC_CSPICTRL);
	reg |= MX1_CSPICTRL_XCH;
	writel(reg, spi_imx->base + MXC_CSPICTRL);
}

static int __maybe_unused mx1_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	unsigned int reg = MX1_CSPICTRL_ENABLE | MX1_CSPICTRL_MASTER;

	reg |= spi_imx_clkdiv_2(spi_imx->spi_clk, config->speed_hz) <<
		MX1_CSPICTRL_DR_SHIFT;
	reg |= config->bpw - 1;

	if (config->mode & SPI_CPHA)
		reg |= MX1_CSPICTRL_PHA;
	if (config->mode & SPI_CPOL)
		reg |= MX1_CSPICTRL_POL;

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int __maybe_unused mx1_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MXC_CSPIINT) & MX1_INTREG_RR;
}

static void __maybe_unused mx1_reset(struct spi_imx_data *spi_imx)
{
	writel(1, spi_imx->base + MXC_RESET);
}

/*
 * These version numbers are taken from the Freescale driver.  Unfortunately it
 * doesn't support i.MX1, so this entry doesn't match the scheme. :-(
 */
static struct spi_imx_devtype_data spi_imx_devtype_data[] __devinitdata = {

#if 0
#ifdef CONFIG_SPI_IMX_VER_IMX1
	[SPI_IMX_VER_IMX1] = {
		.intctrl = mx1_intctrl,
		.config = mx1_config,
		.trigger = mx1_trigger,
		.rx_available = mx1_rx_available,
		.reset = mx1_reset,
		.fifosize = 8,
	},
#endif
#ifdef CONFIG_SPI_IMX_VER_0_0
	[SPI_IMX_VER_0_0] = {
		.intctrl = mx27_intctrl,
		.config = mx27_config,
		.trigger = mx27_trigger,
		.rx_available = mx27_rx_available,
		.reset = spi_imx0_0_reset,
		.fifosize = 8,
	},
#endif
#ifdef CONFIG_SPI_IMX_VER_0_4
	[SPI_IMX_VER_0_4] = {
		.intctrl = mx31_intctrl,
		.config = spi_imx0_4_config,
		.trigger = mx31_trigger,
		.rx_available = mx31_rx_available,
		.reset = spi_imx0_4_reset,
		.fifosize = 8,
	},
#endif
#ifdef CONFIG_SPI_IMX_VER_0_7
	[SPI_IMX_VER_0_7] = {
		.intctrl = mx31_intctrl,
		.config = spi_imx0_7_config,
		.trigger = mx31_trigger,
		.rx_available = mx31_rx_available,
		.reset = spi_imx0_4_reset,
		.fifosize = 8,
	},
#endif
#endif
//#ifdef CONFIG_SPI_IMX_VER_2_3
	[SPI_IMX_VER_2_3] = {
		.intctrl = spi_imx2_3_intctrl,
		.config = spi_imx2_3_config,
		.trigger = spi_imx2_3_trigger,
		.rx_available = spi_imx2_3_rx_available,
		.reset = spi_imx2_3_reset,
		.fifosize = 64,
	},
//#endif

#if 1   //patch spi-slave-dma //gxl 2016.5.31
//#ifdef CONFIG_SPI_IMX_VER_2_3_SLAVE
[SPI_IMX_VER_2_3_SLAVE] = {
		.intctrl = spi_imx2_3_slave_intctrl,
		.config = spi_imx2_3_slave_config,
		.trigger = spi_imx2_3_slave_trigger,
		.rx_available = spi_imx2_3_slave_rx_available,
		.reset = spi_imx2_3_slave_reset,
                //.fifosize = 0,
		.fifosize = 64,
	},
//#endif

#endif

};

static void spi_imx_chipselect(struct spi_device *spi, int is_active)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	//int gpio = spi_imx->chipselect[spi->chip_select];
	//int active = is_active != BITBANG_CS_INACTIVE;
	//int dev_is_lowactive = !(spi->mode & SPI_CS_HIGH);

#if 1   //patch spi-slave-dma //gxl 2016.5.31
        int gpio,active,dev_is_lowactive;
	if(spi_imx->chipselect==NULL) return;		
//#if defined(CONFIG_IMX6_SDP_MISCSPI)						
//	if(spi_imx->master_mode==0) return;
//#else
	gpio = spi_imx->chipselect[spi->chip_select];
	active = is_active != BITBANG_CS_INACTIVE;
	dev_is_lowactive = !(spi->mode & SPI_CS_HIGH);
#endif

	if (gpio < 0)
		return;

	gpio_set_value(gpio, dev_is_lowactive ^ active);
//#endif
}

static void spi_imx_push(struct spi_imx_data *spi_imx)
{
#if 1	
        while (spi_imx->txfifo < spi_imx->devtype_data.fifosize) {
		if (!spi_imx->count)
			break;
		spi_imx->tx(spi_imx);
		spi_imx->txfifo++;
	}
#else
        spi_imx->tx(spi_imx);
#endif
	spi_imx->devtype_data.trigger(spi_imx);
}

#if 1  //patch spi-dma-slave //gxl 2016.5.31
static irqreturn_t spi_dma_isr(int irq, void * dev_id)
{
	struct spi_imx_data *spi_imx = dev_id;
        
	printk("get into function spi_dma_isr!!! \n");
        spi_imx->devtype_data.intctrl(spi_imx, 0);
	return IRQ_HANDLED;
}
#endif

static irqreturn_t spi_imx_isr(int irq, void *dev_id)
{
	struct spi_imx_data *spi_imx = dev_id;

	//printk("get into function spi_imx_isr!!! \n");
        //while (spi_imx->devtype_data.rx_available(spi_imx)) {
        while ((spi_imx->devtype_data.rx_available(spi_imx)))
   	{
		spi_imx->rx(spi_imx);
		spi_imx->txfifo--;
                if(spi_imx->txfifo<0)
		     spi_imx->txfifo=0;

	}

	if (spi_imx->count) {
		
		spi_imx_push(spi_imx);
		return IRQ_HANDLED;
	}

	//if (spi_imx->txfifo) {
        if ((spi_imx->txfifo>0))
   	{
		/* No data left to push, but still waiting for rx data,
		 * enable receive data available interrupt.
		 */
		spi_imx->devtype_data.intctrl(
				spi_imx, MXC_INT_RR);
		return IRQ_HANDLED;
	}
         
        //printk("spi_imx_isr function complete finish!! \n");
	spi_imx->devtype_data.intctrl(spi_imx, 0);
	complete(&spi_imx->xfer_done);

        //printk("spi_imx->devtype_data.intctrl = 0  \n");

	return IRQ_HANDLED;
}
#if 1  //patch spi-dma-slave //gxl 2016.5.31
static int spi_imx_transfer(struct spi_device *spi,
				struct spi_transfer *transfer);
static int spi_imx_dma_transfer(struct spi_device *spi, struct spi_transfer *transfer);
static int spi_imx_dma_init(struct spi_imx_data	*spi_imx);
 
static int spi_imx_choose_th(struct spi_imx_data *spi_imx, int length)
{
	int loop=0;
	int div=0; 
	for(loop=50;loop>=8;loop--){
		if(!(length%loop))	{
			div=loop;	
			goto done;
		}
	}
done:
	return div;
}
#endif


static int spi_imx_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	struct spi_imx_config config;
        int ret=0,fifo_len=0,rc=0;
                   
#if 1  //patch spi-slave-dma //gxl 2015.5.31
		

		//printk("spi_imx_setupxfer: spi_imx->dma_inited = %d,spi_imx->enable_dma = %d\n",spi_imx->dma_inited,spi_imx->enable_dma);
		if((!spi_imx->dma_inited)&&(spi_imx->enable_dma))	/*sdma driver load late*/
		{
			ret=spi_imx_dma_init(spi_imx);
			if(ret){
				spi_imx->enable_dma=0;
			}else{	
				spi_imx->dma_inited=1;
			}		
		}
#endif


		clk_enable(spi_imx->clk);
		config.bpw = t ? t->bits_per_word : spi->bits_per_word;
		config.speed_hz  = t ? t->speed_hz : spi->max_speed_hz;
		config.mode = spi->mode;
		config.cs = spi->chip_select;

		if (!config.speed_hz)
			config.speed_hz = spi->max_speed_hz;
		if (!config.bpw)
			config.bpw = spi->bits_per_word;
		if (!config.speed_hz)
			config.speed_hz = spi->max_speed_hz;

		/* Initialize the functions for transfer */
		if (config.bpw <= 8) {
			spi_imx->rx = spi_imx_buf_rx_u8;
			spi_imx->tx = spi_imx_buf_tx_u8;
                        
		} else if (config.bpw <= 16) {
			spi_imx->rx = spi_imx_buf_rx_u16;
			spi_imx->tx = spi_imx_buf_tx_u16;
		} else if (config.bpw <= 32) {
		//	spi_imx->rx = spi_imx_buf_rx_u32;
		//	spi_imx->tx = spi_imx_buf_tx_u32;
		//} else
		//	BUG();
#if 1   //patch slave-spi-dma //gxl 2015.5.31
		        spi_imx->rx = spi_imx_buf_rx_u32;
			spi_imx->tx = spi_imx_buf_tx_u32;
		} else{
			pr_err("wrong bpw setting");
			ret=-1;
			goto err;	
		}
	       if((t->len%(config.bpw/8))!=0){
			ret=-1;	
			pr_err("transfer len %d not bpw %d times",t->len,config.bpw);
			goto err;
		}	

		spi_imx->bpw=config.bpw;
		//spi_imx->tx_threshold=spi_imx->rx_threshold=spi_imx->devtype_data.fifosize/2;
#if 0               
                spi_imx->tx_threshold= 32;
#endif
                spi_imx->rx_threshold= 0;
		fifo_len=(t->len*8)/config.bpw;
		//printk("spi_imx_setupxfer: config.bpw = %d\n",config.bpw);
        //printk("spi_imx_setupxfer: config.speed_hz = %d\n",config.speed_hz);
		//printk("spi_imx_setupxfer: spi_imx->enable_dma = %d,fifo_len = %d,spi_imx->rx_threshold = %d,spi_imx->tx_threshold = %d\n",spi_imx->enable_dma,fifo_len,spi_imx->rx_threshold,spi_imx->tx_threshold);
		if((spi_imx->enable_dma==0)||(fifo_len<spi_imx->rx_threshold)||(fifo_len<spi_imx->tx_threshold))
		{
			spi_imx->dma_or_pio=0;
			spi_imx->bitbang.txrx_bufs = spi_imx_transfer;
		}
		else 
		{
			rc=spi_imx_choose_th(spi_imx,fifo_len);
			if(!rc){
				spi_imx->dma_or_pio=2;
				//spi_imx->rx_threshold=spi_imx->devtype_data.fifosize/2;
                                spi_imx->rx_threshold = 0;	

			}else{
				spi_imx->rx_threshold=rc-1;
                                spi_imx->rx_threshold = 1;
				spi_imx->dma_or_pio=1;
			}
		        //printk("spi_imx_setupxfer: spi_imx->bitbang.txrx_bufs = spi_imx_dma_transfer\n");
                //printk("spi_imx_setupxfer: spi_imx->rx_threshold = %d , spi_imx->dma_or_pio = %d\n",spi_imx->rx_threshold,spi_imx->dma_or_pio);  
		

                 	spi_imx->bitbang.txrx_bufs = spi_imx_dma_transfer;
		        
		}		
	
#endif
		spi_imx->devtype_data.config(spi_imx, &config);
            
err:
	clk_disable(spi_imx->clk);
	return ret;
}




static int spidev_rxdma_start(struct spi_imx_data *spi_imx)
{
	int ret=0;
      	struct dma_async_tx_descriptor *desc;
      	int dir=DMA_TO_DEVICE;
      	struct scatterlist * sg=NULL;
        unsigned long flag;
      	struct dma_chan *chan;

#if 1

        sg=&spi_imx->rx_sg;

        chan = spi_imx->dma_chan_rx;

        sg_init_one(sg,re_tstdata,4*1024);
        ret=dma_map_sg(spi_imx->dev,sg,1,dir);
     	if(ret!=1){
         	printk("spidev_rxdma_start: dma_map_sg  map failed ret = %d \n",ret);
	 	return -EINVAL;	
      	}
	
        /* Synchronize the DMA transfer with the CPU first
         * so that we see updated contents.
         */

        //dma_sync_sg_for_device(spi_imx->dev, sg, 1, DMA_FROM_DEVICE);

     	desc=chan->device->device_prep_slave_sg(chan,sg,1,DMA_DEV_TO_MEM,0);

	if(!desc){
                printk("device_prep_slave_sg---> Can not init dma descriptor \n");
        }

     	desc->callback=spi_imx_dma_rx_callback;

        desc->callback_param=spi_imx;

     	dmaengine_submit(desc);

     	dma_async_issue_pending(spi_imx->dma_chan_rx);

     	/* enable rx/tx DMA transfer */
     	//spi_imx->devtype_data.trigger(spi_imx);

        //spin_unlock_irqrestore(&spi_imx->lock, flag);
        return 0;  
#else
	spidev_data_frame_t *frame;

	chan = spi_imx->dma_chan_rx;

        frame = list_entry(spi_imx->working_q.next, spidev_data_frame_t, queue);

	sg = &frame->rx_sgl;

	sg->length = 32*1024;

	desc=chan->device->device_prep_slave_sg(chan,sg,1,DMA_DEV_TO_MEM,0);

	if(!desc){
                printk("device_prep_slave_sg---> Can not init dma descriptor \n");
        }

     	desc->callback=spi_imx_dma_rx_callback;

        desc->callback_param=spi_imx;

     	dmaengine_submit(desc);

	dma_async_issue_pending(spi_imx->dma_chan_rx);

     	/* enable rx/tx DMA transfer */
     	//spi_imx->devtype_data.trigger(spi_imx);
#endif     
}

static void rec_process_data_init(void)
{
       u32 i = 0 ;
       
       for( i=0 ;i<1024 ;i++ )
       {
	   rec_process[i] = i;
       }
       
       return;
}
static void spidev_free_mem_rxframe(struct spi_imx_data *spi_imx)
{
      int i = 0;
      u32 *ptr = NULL;
      struct scatterlist *sg;


      for (i = 0; i < 320; i ++) {

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



static u8 trigger_flag = 0;
static u32 rec_finsh_count = 0;

     	
#if 1  //patch spi-slave-dma //gxl 2016.5.31
static void spi_imx_dma_rx_callback(void * data)
{
	struct spi_imx_data *spi_imx=data;
	//struct scatterlist  *sg=&spi_imx->rx_sg;
        u32 val = 0;
        u32 i,ret ;
        spidev_data_frame_t *done_frame;
        struct scatterlist *sg;
        //static u32 rec_finsh_count = 0;

        static u32 finish_num = 0 ;
        
        unsigned long flag;

        static u8 test_flag = 0; 

#if 1
        spin_lock_irqsave(&spi_imx->lock, flag);

    #if 1

        test_flag = !test_flag;
        
        if(test_flag){
            gpio_direction_output(IMX_GPIO_NR(6, 26), 1);
        }
        else{
            gpio_direction_output(IMX_GPIO_NR(6, 26), 0);
        } 

        ret = memcmp (rec_process, re_tstdata, 4096);
        if(ret != 0){
             
            printk("spi_imx_dma_rx_callback----->receive data error !!! ");   
            printk(" rec_finsh_count = %d \n",rec_finsh_count);         
              
            for( i =0 ;i<1024 ;i++ ){
                printk("  %x \r\n",re_tstdata[i]);       
            }
            writel(0, spi_imx->base + 0x08);  //spi control register disable
          
        }

    #endif
        val = readl(spi_imx->base + SPI_IMX2_3_STAT);
        if(val & SPI_IMX2_3_STAT_RO){
		
             printk(" rx FIFO overflow!! \n");
             printk(" rec_finsh_count = %d ,spi_statue_register =0x %x\n",rec_finsh_count,val);   
             writel(0, spi_imx->base + 0x08);  //spi control register disable
        }
	
	//dma_sync_sg_for_cpu(spi_imx->dev, sg, 1, DMA_FROM_DEVICE);  //gxl 2016.6.27

        sg = &spi_imx->rx_sg; 

        dma_unmap_sg(spi_imx->dev,sg,1,DMA_FROM_DEVICE);

        memcpy(backupdata,re_tstdata,4096);
        
        rec_finsh_count ++;

        spidev_rxdma_start(spi_imx);
       
	spin_unlock_irqrestore(&spi_imx->lock, flag);
            

#else
        spin_lock_irqsave(&spi_imx->lock, flag);


        val = readl(spi_imx->base + SPI_IMX2_3_STAT);

        if(val & SPI_IMX2_3_STAT_RO){
		
             printk(" rx FIFO overflow!! ,rec_finsh_count = %d\n",rec_finsh_count);
         
        }	

        //done_frame = list_entry(spi_devdata->working_q.next, spidev_data_frame_t,queue);

        //sg = &done_frame->rx_sgl;

        //dma_sync_sg_for_cpu(spi_imx->dev, sg, 1, DMA_FROM_DEVICE);

             rec_finsh_count++;	  

             list_del(spi_imx->working_q.next);

             if (list_empty(&spi_imx->working_q)){
	          for( i =0; i<320 ;i++){

                  	memcpy(&re_tstdata[i*8*1024],spi_imx->rxsegnums[i].vaddress,spi_imx->rxdma_buf_size);
                  }	
             
	          rec_process_data_init( );
 
                  for( i = 0;i< 2560; i++){

		       ret = memcmp (rec_process, &re_tstdata[i*1024], 4096);
            
                       if(ret != 0){
                 
                            printk(" receive data err---> i = %d  \n",i); 

		            return ; 
                       }

                   }
                  finish_num++;

	          printk(" receive_data is correct  finish_num = %d  \n",finish_num);
    
	          writel(0, spi_imx->base + 0x08);

	          trigger_flag = 0;

                  finish_num = 0;

                  spidev_free_mem_rxframe(spi_imx);   //free memmery ,dma_unmap_sg
#if 0
                  if(spi_imx->dma_chan_rx){
                  
                	dma_release_channel(spi_imx->dma_chan_rx);

                  	spi_imx->dma_chan_rx = NULL;
                  }
#endif

              }

             spidev_rxdma_start(spi_imx);
	
        spin_unlock_irqrestore(&spi_imx->lock, flag);

#endif
       
}
#if 0
static void spi_imx_dma_tx_callback(void * data)
{
	struct spi_imx_data *spi_imx=data;
	struct scatterlist  *sg=&spi_imx->rx_sg;
	dma_unmap_sg(spi_imx->dev,sg,1,DMA_TO_DEVICE);
	spi_imx->dma_number++;
	if(spi_imx->dma_number==2)
		complete(&spi_imx->xfer_done);
}
#endif
static int spi_imx_dma_setup(struct spi_imx_data *spi_imx, struct spi_transfer *transfer,struct dma_chan *chan)
{	
	int dir=DMA_TO_DEVICE,ret;
	struct dma_slave_config slave_config;
	struct scatterlist * sg=NULL;
	void * buf=NULL;
	struct dma_async_tx_descriptor *desc;

	if(chan==spi_imx->dma_chan_rx)
		dir=DMA_FROM_DEVICE;

	if(dir==DMA_FROM_DEVICE){
		slave_config.direction=DMA_DEV_TO_MEM;
		slave_config.src_addr=spi_imx->mapbase + MXC_CSPIRXDATA;
		//slave_config.src_addr_width =spi_imx->bpw/8;
                slave_config.src_addr_width =4;		
		//slave_config.src_maxburst =spi_imx->rx_threshold+1;
                slave_config.src_maxburst =1;
		if(slave_config.src_addr_width==2)							/*FIXME*/
			slave_config.src_maxburst+=slave_config.src_maxburst;
		if(slave_config.src_addr_width==4)
			slave_config.src_maxburst*=4;
		sg=&spi_imx->rx_sg;
		buf=spi_imx->rx_buf;                
                printk("spi_imx_dma_setup: slave_config.src_addr_width = %d,slave_config.src_maxburst = %d \n",slave_config.src_addr_width,slave_config.src_maxburst);
	}else{
#if 0	
                slave_config.direction=DMA_MEM_TO_DEV;
		slave_config.dst_addr=spi_imx->mapbase + MXC_CSPITXDATA;
		slave_config.dst_addr_width = spi_imx->bpw/8;	
		slave_config.dst_maxburst =spi_imx->tx_threshold;
		if(slave_config.src_addr_width==2)							/*FIXME*/
			slave_config.src_maxburst+=slave_config.src_maxburst;
		if(slave_config.src_addr_width==4)
			slave_config.src_maxburst*=4;
		sg=&spi_imx->tx_sg;
		buf=(void *)spi_imx->tx_buf;
#endif	
        }
	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret) {
		pr_err("spi_imx_dma_setup chan %d failed rc %d \n", dir,ret);
                printk("spi_imx_dma_setup: dmaengine_slave_config failed ret = %d \n",ret);
		return -EINVAL;
	}
	printk("spi_imx_dma_setup: spi_imx->dma_or_pio = %d ,dir = %d \n",spi_imx->dma_or_pio,dir);
	if((spi_imx->dma_or_pio==2)&&(dir==DMA_FROM_DEVICE))
	{
		//int fifo_len=(transfer->len*8)/spi_imx->bpw;
		//int mod=fifo_len%(spi_imx->rx_threshold+1);
		//sg_init_one(sg,buf,transfer->len-(mod*spi_imx->bpw/8));

                int fifo_len=(transfer->len*8)/8;
		int mod=fifo_len%(1+1);
		//sg_init_one(sg,buf,transfer->len-(mod*8/8));
                sg_init_one(sg,buf,3);
	}
	else
		sg_init_one(sg,buf,transfer->len);	

	ret=dma_map_sg(spi_imx->dev,sg,1,dir);
	if(ret!=1){
		pr_err("spi_imx_dma_setup map %d failed rc %d \n", dir,ret);
                printk("spi_imx_dma_setup: dma_map_sg  map failed ret = %d \n",ret);
		return -EINVAL;	
	}	
	desc=chan->device->device_prep_slave_sg(chan,sg,1,slave_config.direction,0);
	desc->callback_param=spi_imx;
	if(dir==DMA_FROM_DEVICE){
		spi_imx->rx_desc=desc;
		desc->callback=spi_imx_dma_rx_callback;
                printk("spi_imx_dma_setup: spi_imx_dma_rx_callback \n");
	}
	else{
	#if 0	
                spi_imx->tx_desc=desc;	
		desc->callback=spi_imx_dma_tx_callback;
        #endif     
                printk("spi_imx_dma_setup: desc->callback=spi_imx_dma_tx_callback !!\n");
	}

 	return 0;
 }
#endif


static int spi_imx_transfer(struct spi_device *spi,
				struct spi_transfer *transfer)
{
	int ret;
	printk("spi_imx_transfer===\n\r");
    struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);

	clk_enable(spi_imx->clk);
	spi_imx->tx_buf = transfer->tx_buf;
	spi_imx->rx_buf = transfer->rx_buf;
	spi_imx->count = transfer->len;
	spi_imx->txfifo = 0;

	init_completion(&spi_imx->xfer_done);

	spi_imx_push(spi_imx);

	//spi_imx->devtype_data.intctrl(spi_imx, MXC_INT_TE);
#if 1
	if(spi_imx->master_mode==0){
		spi_imx->devtype_data.intctrl(spi_imx,MXC_INT_RR);
            
        }
	else{
		spi_imx->devtype_data.intctrl(spi_imx, MXC_INT_TE);
        } 
#endif   
       
        printk("spi_imx->master_mode = %d  \n",spi_imx->master_mode);
	//wait_for_completion(&spi_imx->xfer_done);
#if 1   //patch spi_slave_dma //gxl 2016.5.31
//	
	ret=wait_for_completion_timeout(&spi_imx->xfer_done, msecs_to_jiffies(10000));	
	if(!ret){
		pr_err("PIO waited time out\n");
	}
#endif
	clk_disable(spi_imx->clk);

	return transfer->len;
}

#if 1 //patch  spi_slave_dma//gxl 2016.5.31
static int spi_imx_dma_transfer(struct spi_device *spi, struct spi_transfer *transfer)
{
	int ret,fifo_len,mod;
	void * ptr=NULL,*local_buf=NULL;
	struct dma_tx_state state;
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	unsigned long flag;
        int i=0;    
        unsigned char *test_data;

	clk_enable(spi_imx->clk);

        spi_imx->dma_number=0;
	spi_imx->count = transfer->len;
#if 0
	spi_imx->tx_buf = transfer->tx_buf;	
#endif	
        spi_imx->rx_buf = transfer->rx_buf;	

	if(transfer->rx_buf==NULL){ 
		spi_imx->rx_buf=spi_imx->dma_tmp_buf;
		if(transfer->len>SPI_BUF_SIZE){ 
			local_buf=kzalloc(transfer->len,GFP_DMA);
			if(!local_buf) 
				return 0;
			else
				spi_imx->rx_buf=local_buf;
		}
	}
#if 0
	if(transfer->tx_buf==NULL){
		spi_imx->tx_buf=spi_imx->dma_tmp_buf;
		if(transfer->len>SPI_BUF_SIZE){ 
			local_buf=kzalloc(transfer->len,GFP_DMA);
			if(!local_buf) 
				return 0;
			else
				spi_imx->tx_buf=local_buf;	
		}
	}
#endif
	spin_lock_irqsave(&spi_imx->lock, flag);
	ret=spi_imx_dma_setup(spi_imx, transfer,spi_imx->dma_chan_rx);
	if(ret){
		pr_err("spi dma setup rx chan failed");
		spin_unlock_irqrestore(&spi_imx->lock, flag);
                printk("spi_imx_dma_setup rx chan failed ret = %d \n",ret);
		return 0;
	}
#if 0
	ret=spi_imx_dma_setup(spi_imx, transfer,spi_imx->dma_chan_tx);		
	if(ret){
		pr_err("spi dma setup tx chan failed");
		spin_unlock_irqrestore(&spi_imx->lock, flag);
		return 0;
	}
#endif
	spin_unlock_irqrestore(&spi_imx->lock, flag);
		
	init_completion(&spi_imx->xfer_done);
	//spi_imx->devtype_data.trigger(spi_imx);
	//dmaengine_submit(spi_imx->tx_desc);
	dmaengine_submit(spi_imx->rx_desc);

        dma_async_issue_pending(spi_imx->dma_chan_rx);
         /* enable rx/tx DMA transfer */
        spi_imx->devtype_data.trigger(spi_imx);
         
        
//koera	spi_imx->devtype_data.intctrl(spi_imx,MXC_INT_RR|MXC_INT_RF);


#if 1
        udelay(100);
        ret=wait_for_completion_timeout(&spi_imx->xfer_done, msecs_to_jiffies(10000));	
	if(!ret){
		pr_err("waited time out\n");
		spi_imx->dma_chan_rx->device->device_tx_status(spi_imx->dma_chan_rx,
					(dma_cookie_t)NULL, &state);
		pr_err("rx DMA channel received %ud \n ",state.residue);	
	}
#endif	
        
 
        test_data = (unsigned char *)spi_imx->rx_buf;
        for(i=0;i<10;i++)
            printk(" spi_imx->rx_buf[ %d] = %d\n",i,test_data[i]);
	if((spi_imx->dma_or_pio==2))
	{
		fifo_len=(transfer->len*8)/spi_imx->bpw;
		mod=fifo_len%(spi_imx->rx_threshold+1);	
		ptr=spi_imx->rx_buf+transfer->len-(mod*spi_imx->bpw/8);
		
		while((readl(spi_imx->base+SPI_IMX2_3_TEST_REG)>>8)!=mod){
			udelay(50);
		}
		while ((spi_imx->devtype_data.rx_available(spi_imx)))
		{
			unsigned int val = readl(spi_imx->base + MXC_CSPIRXDATA);
			if(spi_imx->bpw==32){
				*(u32 *)ptr=val;
				ptr+=4;
			}
			else if(spi_imx->bpw==16){
				*(u16 *)ptr=val;
				ptr+=2;
			}
			else if(spi_imx->bpw==8){
				*(u8 *)ptr=val;
                                printk(" spi_imx_dma_transfer :receive data val = %d\n",val);
				ptr+=1;
                                

			}
		}
	}
	
	clk_disable(spi_imx->clk);

	if(local_buf!=NULL){
		kfree(local_buf);	
	}
	return transfer->len;	
}
#endif



static int spi_imx_setup(struct spi_device *spi)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	//int gpio = spi_imx->chipselect[spi->chip_select];
#if 1   //patch spi-slave-dma //gxl 2016.5.31       
        int gpio;
        
        if(spi_imx->chipselect==NULL) return 0;

	gpio= spi_imx->chipselect[spi->chip_select];
#endif

	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n", __func__,
		 spi->mode, spi->bits_per_word, spi->max_speed_hz);
//#if defined(CONFIG_IMX6_SDP_MISCSPI)
//	 return 0;
//#else
	if (gpio >= 0)
		gpio_direction_output(gpio, spi->mode & SPI_CS_HIGH ? 0 : 1);

	spi_imx_chipselect(spi, BITBANG_CS_INACTIVE);

	return 0;
//#endif
}

static void spi_imx_cleanup(struct spi_device *spi)
{
}

#if 1   //patch spi_slave_dma //gxl 2016.5.31
static bool imx_spi_filter(struct dma_chan *chan,void *param)
{
	if (!imx_dma_is_general_purpose(chan)){
		return false;
	}
	chan->private= param;
	return true;	
}


static void spi_imx_dma_uninit(struct spi_imx_data	*spi_imx)
{
	if(spi_imx->dma_chan_rx){
		dma_release_channel(spi_imx->dma_chan_rx);
		spi_imx->dma_chan_rx=NULL;
	}
#if 0
	if(spi_imx->dma_chan_tx){
		dma_release_channel(spi_imx->dma_chan_tx);
		spi_imx->dma_chan_tx=NULL;
	}
#endif
	if(spi_imx->dma_tmp_buf){
		kfree(spi_imx->dma_tmp_buf);
		spi_imx->dma_tmp_buf=NULL;		
	}
}

static int spi_imx_dma_init(struct spi_imx_data	*spi_imx)
{
	dma_cap_mask_t mask;
	struct imx_dma_data	dma_data;
	int ret=0;
	unsigned long flag;

	spin_lock_irqsave(&spi_imx->lock, flag);
	
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	dma_data.priority=DMA_PRIO_HIGH;	
	dma_data.dma_request=spi_imx->dma_req_rx;
	//dma_data.peripheral_type =IMX_DMATYPE_CSPI_SP; 
        dma_data.peripheral_type =IMX_DMATYPE_CSPI;
	spi_imx->dma_chan_rx=dma_request_channel(mask,imx_spi_filter,&dma_data);
	if(!spi_imx->dma_chan_rx){
		pr_err("request spi rx dma chan failed");
		goto err;
	}
        //printk("spi_imx_dma_init--->spi_imx->dma_chan_rx = %d\n",spi_imx->dma_chan_rx);

#if 0
	dma_data.priority=DMA_PRIO_LOW;
	dma_data.dma_request=spi_imx->dma_req_tx;
	spi_imx->dma_chan_tx=dma_request_channel(mask,imx_spi_filter,&dma_data);
	if(!spi_imx->dma_chan_tx){
		pr_err("request spi tx dma chan failed");
		goto err;
	}
        printk("spi_imx_dma_init--->spi_imx->dma_chan_tx = %d\n",spi_imx->dma_chan_tx);
#endif		
	spin_unlock_irqrestore(&spi_imx->lock, flag);

#if 0	
	spi_imx->dma_tmp_buf=kzalloc(SPI_BUF_SIZE,GFP_DMA);
	if(!spi_imx->dma_tmp_buf){
		pr_err("alloc spi dma buf failed");
		goto err;	
	}
#endif
        //printk("spi_imx_dma_init--->alloc spi dma buf ok!\n");
	return ret;
err:
	ret=-EINVAL;
	spi_imx_dma_uninit(spi_imx);
	spin_unlock_irqrestore(&spi_imx->lock, flag);
	return ret;
}

#endif

static struct platform_device_id spi_imx_devtype[] = {
#if 0	
        {
		.name = "imx1-cspi",
		.driver_data = SPI_IMX_VER_IMX1,
	}, {
		.name = "imx21-cspi",
		.driver_data = SPI_IMX_VER_0_0,
	}, {
		.name = "imx25-cspi",
		.driver_data = SPI_IMX_VER_0_7,
	}, {
		.name = "imx27-cspi",
		.driver_data = SPI_IMX_VER_0_0,
	}, {
		.name = "imx31-cspi",
		.driver_data = SPI_IMX_VER_0_4,
	}, {
		.name = "imx35-cspi",
		.driver_data = SPI_IMX_VER_0_7,
	}, {
		.name = "imx50-cspi",
		.driver_data = SPI_IMX_VER_0_7,
	}, {
		.name = "imx51-cspi",
		.driver_data = SPI_IMX_VER_0_7,
	}, {
		.name = "imx51-ecspi",
		.driver_data = SPI_IMX_VER_2_3,
	}, {
		.name = "imx53-cspi",
		.driver_data = SPI_IMX_VER_0_7,
	}, {
		.name = "imx53-ecspi",
		.driver_data = SPI_IMX_VER_2_3,
	}, 
#endif  
        {
		.name = "imx6q-ecspi",
		.driver_data = SPI_IMX_VER_2_3,
	}, 
#if 1 //patch spi_slave_dma //gxl 2016.5.31
        {
		.name = "imx6q-ecspi-slave",
		.driver_data = SPI_IMX_VER_2_3_SLAVE,
	}, 
#endif     
       {
		/* sentinel */
	}
};

static int spidev_alloc_mem_rxframe(struct spi_imx_data *spi_devdata )
{
	int i = 0;
	struct scatterlist *sg;
	int ret;


	if (!spi_devdata)
		return -1;

	spi_devdata->rxdma_buf_size = 32*1024;
	spi_devdata->num_rxdma_bufs = 320;

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

	}

	printk("spidev_alloc_mem_rxframe: step--->2 \n"); 
	return 0;
}


static int spidev_init_rxsegnums_buf(struct spi_imx_data *spi_devdata)
{
	int i = 0;

	unsigned long flag;

	if (!spi_devdata)
		return -1;

        printk("go to spidev_init_rxsegnums_buf !!\n");

	for (i = 0; i < 320; i++) {
		spi_devdata->rxsegnums[i].buffer.offset = spi_devdata->rxsegnums[i].paddress;
		spi_devdata->rxsegnums[i].buffer.index = i;
		spi_devdata->rxsegnums[i].buffer.flags = 0x01;         //SPIDEV_BUF_FLAG_MAPPED
		spi_devdata->rxsegnums[i].buffer.length = 32*1024;
		spi_devdata->rxsegnums[i].index = i;
                
                //printk("go to spidev_init_rxsegnums_buf --->one ,%d\n",i);

		list_add_tail(&spi_devdata->rxsegnums[i].queue, &spi_devdata->working_q);

                //printk("go to spidev_init_rxsegnums_buf ---->two,%d \n",i);                 

		spi_devdata->rxsegnums[i].buffer.flags |= 0x02;       //SPIDEV_BUF_FLAG_QUEUED
	}
	printk("spidev_init_rxsegnums_buf: step--->3 \n");

return 0;
}

static int spidev_rxstreamon(struct spi_imx_data *spi_imx)
{
       	int ret=0;
      	struct dma_slave_config slave_config;
      	struct dma_async_tx_descriptor *desc;
      	int dir=DMA_TO_DEVICE;
      	struct scatterlist * sg=NULL;
      	struct dma_chan *chan;
        unsigned long flag;
	spidev_data_frame_t *frame;
        u32 temp_reg = 0;
        


      	struct spi_imx_config config;
      	int ret_err=0,buf_len=0,rc=0;
                   
		
        
	ret=spi_imx_dma_init(spi_imx);
	if(ret== 0)
	{
	     printk("spi_imx_dma_init  finish ok! \n");
	}
	else{
		return ret;
	}

	chan = spi_imx->dma_chan_rx;

	slave_config.direction=DMA_DEV_TO_MEM;
	slave_config.src_addr=spi_imx->mapbase + MXC_CSPIRXDATA;
	slave_config.src_addr_width =4;	//(bpw/8)	
	slave_config.src_maxburst = 32*4;   //(rx_threshold +1) * src_addr_width   /*FIXME*/


     	ret = dmaengine_slave_config(chan, &slave_config);
     	if (ret) {
		printk("spi_imx_dma_setup: dmaengine_slave_config failed ret = %d \n",ret);
		return -EINVAL;
     	}

	config.bpw = 32;
	config.speed_hz  = 6500000;//15000000;
	config.mode = 0;
#if 1
	config.cs = 0;
#else
        config.cs = 1;
#endif
	spi_imx->bpw = 32;               
	spi_imx->rx_threshold = 31;
	  
	spin_lock_irqsave(&spi_imx->lock, flag);
								
	spi_imx->devtype_data.config(spi_imx, &config);
	
	spin_unlock_irqrestore(&spi_imx->lock, flag);
       
#if 1

	sg=&spi_imx->rx_sg;  

	spin_lock_irqsave(&spi_imx->lock, flag);

        sg_init_one(sg,re_tstdata,4*1024);
	
     	ret=dma_map_sg(spi_imx->dev,sg,1,dir);
     	if(ret==0){
         	printk("spi_imx_dma_setup: dma_map_sg  map failed ret = %d \n",ret);
	 	return -EINVAL;	
      	}

        //dma_sync_sg_for_cpu(spi_imx->dev, sg, 1, DMA_FROM_DEVICE);  //gxl 2016.6.27

	//dma_sync_sg_for_device(spi_imx->dev, sg, 1, DMA_FROM_DEVICE);

     	desc=chan->device->device_prep_slave_sg(chan,sg,1,slave_config.direction,0);
        
        if(!desc){
                printk("device_prep_slave_sg---> Can not init dma descriptor \n");
        }
        spin_unlock_irqrestore(&spi_imx->lock, flag);
           	
     	desc->callback=spi_imx_dma_rx_callback;
        
	desc->callback_param=spi_imx;

        spin_lock_irqsave(&spi_imx->lock, flag); 
        
     	dmaengine_submit(desc);

     	//dma_async_issue_pending(spi_imx->dma_chan_rx);
         
     	/* enable rx/tx DMA transfer */
     	spi_imx->devtype_data.trigger(spi_imx);

        spin_unlock_irqrestore(&spi_imx->lock, flag);

	rec_process_data_init( );

#else
       INIT_LIST_HEAD(&spi_imx->ready_q);
       INIT_LIST_HEAD(&spi_imx->done_q);
       INIT_LIST_HEAD(&spi_imx->working_q);


        //spin_lock_irqsave(&spi_imx->lock, flag);

        printk("spidev_rxstreamon: step--->1 \n"); 	
 
        spidev_alloc_mem_rxframe(spi_imx );

	spidev_init_rxsegnums_buf(spi_imx);

        printk("spidev_rxstreamon: step--->4 \n");

	frame = list_entry(spi_imx->working_q.next, spidev_data_frame_t, queue);

        printk("spidev_rxstreamon: step--->5 \n");
        
	sg = &frame->rx_sgl;

        sg->length = 32*1024;

	desc=chan->device->device_prep_slave_sg(chan,sg,1,slave_config.direction,0);
        
        if(!desc){
                printk("device_prep_slave_sg---> Can not init dma descriptor \n");
        }

        printk("spidev_rxstreamon: step--->6 \n");

	desc->callback=spi_imx_dma_rx_callback;
        
	desc->callback_param= spi_imx;

        spin_lock_irqsave(&spi_imx->lock, flag); 
        
     	dmaengine_submit(desc);

	dma_async_issue_pending(spi_imx->dma_chan_rx);
         
     	/* enable rx/tx DMA transfer */
     	spi_imx->devtype_data.trigger(spi_imx);
      
        spin_unlock_irqrestore(&spi_imx->lock, flag);

        printk("spidev_rxstreamon: step--->7 \n");

#endif       
}

#if 1   //gxl 2016
static irqreturn_t enabledclock_and_startdma(int irq, void *_data)
{
   
       struct spi_imx_data *spi_imx = _data;
       unsigned int reg_statue;
       u32 reg_dma;
       u32 reg_spiconfig;
       u32 reg_ctl;
       unsigned long flag;
       static u8 boot_nums = 0;
       
       if(trigger_flag == 0){
	       
	       printk("enabledclock_and_startdma:spi_imx->clk = %d\n",spi_imx->clk); 
  
               spin_lock_irqsave(&spi_imx->lock, flag);

               
               if(boot_nums == 0)
	            clk_enable(spi_imx->clk); 
              
               boot_nums++;

               spin_unlock_irqrestore(&spi_imx->lock, flag);
	       spidev_rxstreamon(spi_imx);     
	 	    	       
	       reg_statue = readl(spi_imx->base+0x18); //SPI_IMX2_3_STAT
	       reg_dma = readl(spi_imx->base+0x14);
	       reg_spiconfig = readl(spi_imx->base+0x0C); //SPI_CONFIG_REGSITER
               reg_ctl = readl(spi_imx->base+0x08); 
	       printk("reg_statue = %d ,reg_spiconfig = %x \n",reg_statue,reg_spiconfig);
               printk("reg_ctl = %x\n",reg_ctl );
               printk("reg_dma = %x\n",reg_dma );
               trigger_flag++; 

       }
       
    return IRQ_HANDLED;
}


static irqreturn_t check_dma_callback_nums(int irq, void *_data)
{
   
       struct spi_imx_data *spi_imx = _data;
 
       
       printk("check_dma_callback_nums = %d \n",rec_finsh_count);
       
       return IRQ_HANDLED;
}


#endif

static int __devinit spi_imx_probe(struct platform_device *pdev)
{
	struct spi_imx_master *mxc_platform_info;
	struct spi_master *master;
	struct spi_imx_data *spi_imx;
	struct resource *res;
	int i, ret;

	mxc_platform_info = dev_get_platdata(&pdev->dev);
	if (!mxc_platform_info) {
		dev_err(&pdev->dev, "can't get the platform data\n");
		return -EINVAL;
	}

	master = spi_alloc_master(&pdev->dev, sizeof(struct spi_imx_data));
	if (!master)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);

	master->bus_num = pdev->id;
	master->num_chipselect = mxc_platform_info->num_chipselect;

	spi_imx = spi_master_get_devdata(master);
	spi_imx->bitbang.master = spi_master_get(master);
	//spi_imx->chipselect = mxc_platform_info->chipselect;

	//for (i = 0; i < master->num_chipselect; i++) {

#if 1  //patch spi_dma_slave //gxl 2016.5.31
	spi_imx->chipselect  =mxc_platform_info->chipselect;
	spi_imx->master_mode =mxc_platform_info->master_mode;
	//spi_imx->enable_dma = 1;
	spi_imx->dma_req_rx  =mxc_platform_info->rx_dma_req;
	//spi_imx->dma_req_tx  =mxc_platform_info->tx_dma_req;
	spi_imx->dev=&pdev->dev;
	spi_imx->dma_inited=0;
	spin_lock_init(&spi_imx->lock);

	printk("spi_imx_probe: spi_imx->dma_req_rx = %d \n",spi_imx->dma_req_rx);
//#if defined(CONFIG_IMX6_SDP_MISCSPI)
//#else
	for (i = 0;(spi_imx->chipselect)&&(i < master->num_chipselect); i++) {
#endif

		if (spi_imx->chipselect[i] < 0)
			continue;
		ret = gpio_request(spi_imx->chipselect[i], DRIVER_NAME);
		if (ret) {
			while (i > 0) {
				i--;
				if (spi_imx->chipselect[i] >= 0)
					gpio_free(spi_imx->chipselect[i]);
			}
			dev_err(&pdev->dev, "can't get cs gpios\n");
			goto out_master_put;
		}
	}
//#endif

//#ifdef SLAVA_DMA_REC
	spi_imx->bitbang.chipselect = spi_imx_chipselect;
	spi_imx->bitbang.setup_transfer = spi_imx_setupxfer;
	spi_imx->bitbang.txrx_bufs = spi_imx_transfer;
	spi_imx->bitbang.master->setup = spi_imx_setup;
	spi_imx->bitbang.master->cleanup = spi_imx_cleanup;
	spi_imx->bitbang.master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

	init_completion(&spi_imx->xfer_done);
//#endif

#if 1  //patch spi_dma_slave //gxl 2016.5.31 
#if defined(CONFIG_SPI_IMX_VER_2_3_SLAVE)
        printk("spi_imx_probe: spi_imx->master_mode = %d \n",spi_imx->master_mode);
	if(spi_imx->master_mode==0){
                printk("spi_imx_probe: spi_imx->master_mode == 0 \n");
                dev_info(&pdev->dev,"IMX6 spi controller use slave mode");	
		//spi_imx->devtype_data=spi_imx_devtype_data[SPI_IMX_VER_2_3_SLAVE];
                spi_imx->devtype_data=spi_imx_devtype_data[1];
	}
	else
#endif
	{
		//printk("pdev->id_entry->driver_data= %d\n",pdev->id_entry->driver_data); 
                spi_imx->devtype_data =
			spi_imx_devtype_data[0];

		dev_info(&pdev->dev,"IMX6 spi controller mode %d\n",spi_imx->master_mode);	
	}
#endif
	//printk("find  NULL pointe step--->1 !!\n"); 

        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "can't get platform resource\n");
		ret = -ENOMEM;
		goto out_gpio_free;
	}
        //printk("find  NULL pointe step--->2 !!\n");
	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		ret = -EBUSY;
		goto out_gpio_free;
	}
        //printk("find  NULL pointe step--->3 !!\n");
	spi_imx->base = ioremap(res->start, resource_size(res));
	if (!spi_imx->base) {
		ret = -EINVAL;
		goto out_release_mem;
	}
        //printk("find  NULL pointe step--->4 !!\n");
#if 1   //patch spi_dma_slave //gxl 2016.5.31
        spi_imx->mapbase=res->start;
        printk("spi_imx->base = %d \n",spi_imx->base);
#endif

//#ifdef SLAVA_DMA_REC
	spi_imx->irq = platform_get_irq(pdev, 0);
	if (spi_imx->irq < 0) {
		ret = -EINVAL;
		goto out_iounmap;
	}
        printk("spi_imx->irq = %d \n",spi_imx->irq);
        //printk("find  NULL pointe step--->5 !!\n");
	ret = request_irq(spi_imx->irq, spi_imx_isr, 0, DRIVER_NAME, spi_imx);
	if (ret) {
		dev_err(&pdev->dev, "can't get irq%d: %d\n", spi_imx->irq, ret);
		goto out_iounmap;
	}
//#endif
        //printk("find  NULL pointe step--->6 !!\n");
	spi_imx->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(spi_imx->clk)) {
		dev_err(&pdev->dev, "unable to get clock\n");
		ret = PTR_ERR(spi_imx->clk);
		goto out_free_irq;
	}
        printk("spi_imx->clk = %d \n",spi_imx->clk);
        //printk("find  NULL pointe step--->7 !!\n");
	clk_enable(spi_imx->clk);
	spi_imx->spi_clk = clk_get_rate(spi_imx->clk);
        printk("spi_imx->spi_clk = %d \n",spi_imx->spi_clk);
        //printk("find  NULL pointe step--->8 !!\n");
	spi_imx->devtype_data.reset(spi_imx);

        //printk("find  NULL pointe step--->11 !!\n");
	spi_imx->devtype_data.intctrl(spi_imx, 0);
        //printk("find  NULL pointe step--->12 !!\n");
                
//#ifdef SLAVA_DMA_REC
	ret = spi_bitbang_start(&spi_imx->bitbang);
	if (ret) {
		dev_err(&pdev->dev, "bitbang start failed with %d\n", ret);
		goto out_clk_put;
	}
//#endif
        //printk("find  NULL pointe step--->13 !!\n");
	clk_disable(spi_imx->clk);
        

   
#if 0    //gxl add 2--3ms acting INT   2016.5.16
 
     gpio_request(IMX_GPIO_NR(1, 5), "INPUT_KEY");
     
     gpio_direction_input(IMX_GPIO_NR(1, 5));
 
     ret = request_threaded_irq(gpio_to_irq(IMX_GPIO_NR(1, 5)),
                            NULL, enabledclock_and_startdma,
                            IRQF_TRIGGER_FALLING,
                            "INPUT_KEY_Status", spi_imx);

     if (ret) {
           printk("int request fail!!\n");
      
     }

     gpio_request(IMX_GPIO_NR(1, 4), "INT_CHECK");
     
     gpio_direction_input(IMX_GPIO_NR(1, 4));
 
     ret = request_threaded_irq(gpio_to_irq(IMX_GPIO_NR(1, 4)),
                            NULL, check_dma_callback_nums,
                            IRQF_TRIGGER_FALLING,
                            "COUNT_CALLBACK", spi_imx);

     if (ret) {
           printk("int request fail!!\n");
      
     }


     gpio_request(IMX_GPIO_NR(6, 26), "Test_INT_Internel");

     gpio_direction_output(IMX_GPIO_NR(6, 26),1);

     gpio_set_value(IMX_GPIO_NR(6, 26), 1);

     //gpio_request(IMX_GPIO_NR(2, 27), "spi_cs");

     //gpio_direction_input(IMX_GPIO_NR(2, 27));
#endif
       
	dev_info(&pdev->dev, "probed\n");

	return ret;

out_clk_put:
	clk_disable(spi_imx->clk);
	clk_put(spi_imx->clk);
out_free_irq:
	free_irq(spi_imx->irq, spi_imx);
out_iounmap:
	iounmap(spi_imx->base);
out_release_mem:
	release_mem_region(res->start, resource_size(res));
out_gpio_free:
	//for (i = 0; i < master->num_chipselect; i++)
#if 1  //patch spi_slave_dma //gxl 2016.5.31
       for (i = 0; spi_imx->chipselect&&(i < master->num_chipselect); i++)
#endif
		if (spi_imx->chipselect[i] >= 0)
			gpio_free(spi_imx->chipselect[i]);
out_master_put:
	spi_master_put(master);
	kfree(master);
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int __devexit spi_imx_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct spi_imx_data *spi_imx = spi_master_get_devdata(master);
	int i;

#if 1  //patch spi_dma_slave //gxl 2016.5.31
        if(spi_imx->dma_inited){
		spi_imx_dma_uninit(spi_imx);
	}
#endif

	spi_bitbang_stop(&spi_imx->bitbang);
	clk_enable(spi_imx->clk);
	writel(0, spi_imx->base + MXC_CSPICTRL);
	clk_disable(spi_imx->clk);
	clk_put(spi_imx->clk);
	free_irq(spi_imx->irq, spi_imx);
	iounmap(spi_imx->base);

	//for (i = 0; i < master->num_chipselect; i++)
#if 1 //patch spi_slave_dma //gxl 2016.5.31
      for (i = 0;spi_imx->chipselect&&(i < master->num_chipselect); i++)
#endif
		if (spi_imx->chipselect[i] >= 0)
			gpio_free(spi_imx->chipselect[i]);

	spi_master_put(master);

	release_mem_region(res->start, resource_size(res));

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver spi_imx_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   },
	.id_table = spi_imx_devtype,
	.probe = spi_imx_probe,
	.remove = __devexit_p(spi_imx_remove),
};

static int __init spi_imx_init(void)
{
	return platform_driver_register(&spi_imx_driver);
}

static void __exit spi_imx_exit(void)
{
	platform_driver_unregister(&spi_imx_driver);
}

subsys_initcall(spi_imx_init);
module_exit(spi_imx_exit);

MODULE_DESCRIPTION("SPI Master Controller driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix");
MODULE_LICENSE("GPL");
