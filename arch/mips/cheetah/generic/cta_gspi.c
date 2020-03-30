/*
 * spi_camelot.c - Camelot Generic SPI master controller driver
 * edit from spi_bitbang.c and spi_gpio.c
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_camelot.h>

#define DRIVER_NAME	"camelot_generic_spi"

#define gspi_log(...)  
//#define gspi_log printk
#define cml_gspi_debug(...)
//#define cml_gspi_debug printk

struct spi_camelot_platform_data {
	int num_chipselect;
};

struct spi_camelot {
	struct spi_camelot_platform_data	pdata;
	struct platform_device		*pdev;
	
	struct workqueue_struct	*workqueue;
	struct work_struct	work;

	spinlock_t		lock;
	struct list_head	queue;
	u8			busy;

	struct spi_master	*master;

	/* setup_transfer() changes clock and/or wordsize to match settings
	 * for this transfer; zeroes restore defaults from spi_device.
	 */
	int	(*setup_transfer)(struct spi_device *spi, struct spi_transfer *t);

	void	(*chipselect)(struct spi_device *spi, int is_on);
#define	SPI_CAMELOT_CS_ACTIVE	1	/* normally nCS, active low */
#define	SPI_CAMELOT_INACTIVE	0

	int	(*txrx_bufs)(struct spi_device *spi, struct spi_transfer *t);
};
static unsigned int gspi_cbase[CML_GSPI_CHIPSELECT_NUM]={MI_BASE+0x30,MI_BASE+0x40};
static inline void SET_REG(unsigned int addr, unsigned int mask, unsigned int value)
{
    unsigned int temp;
    temp = SPIREG(addr) & mask;
    SPIREG(addr) = temp | value;
			iob();
}
static inline void spi_keep_cs(unsigned int bank, int keep)
{
    if (keep == 1 || keep == 0)
    {
        SET_REG(gspi_cbase[bank] + GSPI_CSR, ~(1<<6), (keep<<6));
    }
}
static inline void spi_set_len(unsigned int bank, unsigned int len)
{
    SPIREG(gspi_cbase[bank] + GSPI_CR) = len;
			iob();
}
static inline int spi_ready(unsigned int bank)
{
    unsigned int i,rc;
    for (i=0; i<SPI_RX_WAIT_LOOP; i++)
    {
        rc = SPIREG(gspi_cbase[bank] + GSPI_CSR);
        if (((rc & SPI_TXEMPTY) != 0) && ((rc & SPI_RXNONEMPTY) == 0))
            return 1;
    }
    gspi_log("SPI wait ready timeout\n");
    return 0;
}
int cml_gspi_rx(unsigned int bank, unsigned int data_bit_len, unsigned char * rx_buf)
{
    // maximum SPI_Length = 2^16 - 1 bits
    unsigned int i;
    volatile unsigned int * temp;
    // check status
    if (!spi_ready(bank))
        return -2;
    // set SKIP and SPI_Length
    spi_set_len(bank, data_bit_len);
    gspi_log("bit_len=%d\n",data_bit_len);
    // read all data from SPI Data Register
    for (i=0; i<(data_bit_len +7)/8; i+=4)
    {
        temp = (int *)(rx_buf + i);
        *temp = SPIREG(gspi_cbase[bank] + GSPI_DR);
        gspi_log("SPI read =0x%08x\n",*temp);
    }
    return 0;
}

int cml_gspi_tx(unsigned int bank, unsigned int data_bit_len, unsigned char * tx_buf)
{
    // maximum SPI_Length = 2^16 - 1 bits
    unsigned int i,j,rc;
    volatile unsigned int temp;
    // check status
    if (!spi_ready(bank))
        return -2;
    // set SKIP and SPI_Length
    spi_set_len(bank, data_bit_len);
    gspi_log("bit_len=%d\n",data_bit_len);
    // write all data into SPI_DR
    for (i=0; i<(data_bit_len +7)/8; i+=4)
    {
        // wait TX buffer isn't full
        for (j=0; j<SPI_TX_WAIT_LOOP; j++)
        {
            rc = SPIREG(gspi_cbase[bank] + GSPI_CSR);
            if ((rc & SPI_TXFULL) == 0)
                break;
        }
        if (j == SPI_TX_WAIT_LOOP)
            return -1;
        temp = *(int *)(tx_buf + i);
        SPIREG(gspi_cbase[bank] + GSPI_DR) = temp;
			iob();
		gspi_log("SPI write =0x%08x\n",temp);
     }
		return 0;
}

static void cml_gspi_init(unsigned int bank, unsigned int init_val)
{
    // set serial mode for bank
    SPIREG(MI_BASE+SROMEN) &= ~(2<<bank); //For GSPI ,first chipselect is second bank
			iob();
	gspi_log("cml_gspi_init: bank=%d init_val=0x%x reg_base = 0x%x\n",bank,init_val,MI_BASE);
    // SPI init SPI_CLK_width,SPI_CLK_Park,SPI_Mode,SPI Rising/Falling Sample Data
    SET_REG(gspi_cbase[bank] + GSPI_CSR, SPI_INIT_MASK, init_val);
    // reset SPI state machine
    SET_REG(gspi_cbase[bank] + GSPI_CSR, ~(1<<7), (1<<7));
    SET_REG(gspi_cbase[bank] + GSPI_CSR, ~(1<<7), (0<<7));
}

static void cml_gspi_chipselect(struct spi_device *spi, int is_active)
{
	spi_keep_cs(spi->chip_select, is_active);
}
/*----------------------------------------------------------------------*/

/*
 * FIRST PART (OPTIONAL):  word-at-a-time spi_transfer support.
 * Use this for GPIO or shift-register level hardware APIs.
 *
 * spi_bitbang_cs is in spi_device->controller_state, which is unavailable
 * to glue code.  These bitbang setup() and cleanup() routines are always
 * used, though maybe they're called from controller-aware code.
 *
 * chipselect() and friends may use use spi_device->controller_data and
 * controller registers as appropriate.
 */

struct spi_bitbang_cs {
	unsigned	nsecs;	/* (clock cycle time)/2 */
	unsigned	(*txrx_bufs)(struct spi_device *, struct spi_transfer *);
};

static unsigned bitbang_txrx_1(
	struct spi_device	*spi,
	struct spi_transfer	*t
) {
	unsigned		bit_count = t->len;
	unsigned		bits; 
	const u32	*tx = t->tx_buf;
	u32			*rx = t->rx_buf;

	while (likely(bit_count > 0)) {
		u32	word = 0;
		bits = (bit_count > 32) ? 32 : bit_count;
		gspi_log("----- bitbang_txrx_1 ----- count = %d\n",bit_count);
		if (tx)
		{
			word = *tx++;
			cml_gspi_tx(spi->chip_select,bits,(unsigned char*)&word);
		}
		else if (rx)
		{
			cml_gspi_rx(spi->chip_select,bits,(unsigned char*)rx);
			word = *rx++;
			gspi_log("cml_gspi_rx = 0x%x\n",word);
		}
		bit_count -= bits;
	}
	return t->len - bit_count;
}

static unsigned bitbang_txrx_8(
	struct spi_device	*spi,
	struct spi_transfer	*t
) {
	unsigned		bits = spi->bits_per_word;
	unsigned		count = t->len;
	const u8		*tx = t->tx_buf;
	u8			*rx = t->rx_buf;

	while (likely(count > 0)) {
		u8		word = 0;
	gspi_log("----- bitbang_txrx_8 ----- count = %d\n",count);
		if (tx)
		{
			word = *tx++;
			cml_gspi_tx(spi->chip_select,bits,&word);
		}
		else if (rx)
		{
			cml_gspi_rx(spi->chip_select,bits,rx);
			word = *rx++;
			gspi_log("cml_gspi_rx = 0x%x\n",word);
		}
		count -= 1;
	}
	return t->len - count;
}

static unsigned bitbang_txrx_16(
	struct spi_device	*spi,
	struct spi_transfer	*t
) {
	unsigned		bits = spi->bits_per_word;
	unsigned		count = t->len;
	const u16		*tx = t->tx_buf;
	u16			*rx = t->rx_buf;

	while (likely(count > 1)) {
		u16		word = 0;
	gspi_log("----- bitbang_txrx_16 ----- count = %d\n",count);
		if (tx)
		{
	/* endian convert for Marvell 8686 */
#if defined(CONFIG_LIBERTAS_SPI_MODULE) | defined(CONFIG_LIBERTAS_SPI)
			word = cpu_to_le16(*tx++);
#else
			word = *tx++;
#endif
			cml_gspi_tx(spi->chip_select,bits,(unsigned char*)&word);
		}
		else if (rx)
		{
			cml_gspi_rx(spi->chip_select,bits,(unsigned char*)rx);
	/* endian convert for Marvell 8686 */
#if defined(CONFIG_LIBERTAS_SPI_MODULE) | defined(CONFIG_LIBERTAS_SPI)
			*rx = cpu_to_le16(*rx);
#endif
			word = *rx++;
			gspi_log("cml_gspi_rx = 0x%x\n",word);
		}
		count -= 2;
	}
	return t->len - count;
}

static unsigned bitbang_txrx_32(
	struct spi_device	*spi,
	struct spi_transfer	*t
) {
	unsigned		bits = spi->bits_per_word;
	unsigned		count = t->len;
	const u32		*tx = t->tx_buf;
	u32			*rx = t->rx_buf;

	while (likely(count > 3)) {
		u32		word = 0;
	gspi_log("----- bitbang_txrx_32 ----- count = %d\n",count);
		if (tx)
		{
	/* endian convert for Marvell 8686 */
#if defined(CONFIG_LIBERTAS_SPI_MODULE) | defined(CONFIG_LIBERTAS_SPI)
			word = cpu_to_le32(*tx++);
#else
			word = *tx++;
#endif
			cml_gspi_tx(spi->chip_select,bits,(unsigned char*)&word);
		}
		else if (rx)
		{
			cml_gspi_rx(spi->chip_select,bits,(unsigned char*)rx);
	/* endian convert for Marvell 8686 */
#if defined(CONFIG_LIBERTAS_SPI_MODULE) | defined(CONFIG_LIBERTAS_SPI)
			*rx = cpu_to_le32(*rx);
#endif
			word = *rx++;
			gspi_log("cml_gspi_rx = 0x%x\n",word);
		}
		count -= 4;
	}
	return t->len - count;
}

/*
 * This bitbanging SPI master driver should help make systems usable
 * when a native hardware SPI engine is not available, perhaps because
 * its driver isn't yet working or because the I/O pins it requires
 * are used for other purposes.
 *
 * platform_device->driver_data ... points to spi_gpio
 *
 * spi->controller_state ... reserved for bitbang framework code
 * spi->controller_data ... holds chipselect GPIO
 *
 * spi->master->dev.driver_data ... points to spi_gpio->bitbang
 */

int spi_camelot_setup_transfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct spi_bitbang_cs	*cs = spi->controller_state;
	u8			bits_per_word;
	u32			hz;

	if (t) {
		bits_per_word = t->bits_per_word;
		hz = t->speed_hz;
	} else {
		bits_per_word = 0;
		hz = 0;
	}
	/* spi_transfer level calls that work per-word */
	if (!bits_per_word)
		bits_per_word = spi->bits_per_word;
	cml_gspi_debug("----- setup_transfer ------ bits_per_word=0x%x\n", bits_per_word);
	if (bits_per_word == 1)
		cs->txrx_bufs = bitbang_txrx_1;
	else if (bits_per_word <= 8)
		cs->txrx_bufs = bitbang_txrx_8;
	else if (bits_per_word <= 16)
		cs->txrx_bufs = bitbang_txrx_16;
	else if (bits_per_word <= 32)
		cs->txrx_bufs = bitbang_txrx_32;
	else
		return -EINVAL;

	/* nsecs = (clock period)/2 */
	if (!hz)
		hz = spi->max_speed_hz;
	if (hz) {
		cs->nsecs = (1000000000/2) / hz;
	cml_gspi_debug("----- setup_transfer ----- hz=%d cs->nsecs=0x%x MAX_UDELAY_MS =0x%x\n",hz,cs->nsecs,(MAX_UDELAY_MS*1000*1000));
		if (cs->nsecs > (MAX_UDELAY_MS * 1000 * 1000))
			return -EINVAL;
	}

	return 0;
}


/*----------------------------------------------------------------------*/

/*
 * SECOND PART ... simple transfer queue runner.
 *
 * This costs a task context per controller, running the queue by
 * performing each transfer in sequence.  Smarter hardware can queue
 * several DMA transfers at once, and process several controller queues
 * in parallel; this driver doesn't match such hardware very well.
 *
 * Drivers can provide word-at-a-time i/o primitives, or provide
 * transfer-at-a-time ones to leverage dma or fifo hardware.
 */
static void spi_camelot_work(struct work_struct *work)
{
	struct spi_camelot *spi_camelot = container_of(work, struct spi_camelot, work);
	unsigned long flags;
	int do_setup = -1;
	int (*setup_transfer)(struct spi_device *, struct spi_transfer *);

	setup_transfer = spi_camelot->setup_transfer;

	spin_lock_irqsave(&spi_camelot->lock, flags);
	spi_camelot->busy = 1;
	while (!list_empty(&spi_camelot->queue)) {
		struct spi_message	*m;
		struct spi_device	*spi;
		unsigned		nsecs;
		struct spi_transfer	*t = NULL;
		unsigned		cs_change;
		int			status;

		m = container_of(spi_camelot->queue.next, struct spi_message,
				queue);
		list_del_init(&m->queue);
		spin_unlock_irqrestore(&spi_camelot->lock, flags);

		/* FIXME this is made-up ... the correct value is known to
		 * word-at-a-time bitbang code, and presumably chipselect()
		 * should enforce these requirements too?
		 */
		nsecs = 100;

		spi = m->spi;
		cs_change = 1;
		status = 0;

		list_for_each_entry (t, &m->transfers, transfer_list) {

			/* override speed or wordsize? */
			if (t->speed_hz || t->bits_per_word)
				do_setup = 1;

			/* init (-1) or override (1) transfer params */
			if (do_setup != 0) {
				if (!setup_transfer) {
					status = -ENOPROTOOPT;
					break;
				}
				cml_gspi_debug("----- spi_camelot_work -----\n");
				status = setup_transfer(spi, t);
				cml_gspi_debug("----- spi_camelot_work ----1\n");
				if (status < 0)
					break;
			}

			/* set up default clock polarity, and activate chip;
			 * this implicitly updates clock and spi modes as
			 * previously recorded for this device via setup().
			 * (and also deselects any other chip that might be
			 * selected ...)
			 */
			if (cs_change) {
				spi_camelot->chipselect(spi, SPI_CAMELOT_CS_ACTIVE);
				ndelay(nsecs);
			}
			cs_change = t->cs_change;
			if (!t->tx_buf && !t->rx_buf && t->len) {
				status = -EINVAL;
				break;
			}

			/* transfer data.  the lower level code handles any
			 * new dma mappings it needs. our caller always gave
			 * us dma-safe buffers.
			 */
			if (t->len) {
				/* REVISIT dma API still needs a designated
				 * DMA_ADDR_INVALID; ~0 might be better.
				 */
				if (!m->is_dma_mapped)
					t->rx_dma = t->tx_dma = 0;
				status = spi_camelot->txrx_bufs(spi, t);
			}
			if (status > 0)
				m->actual_length += status;
			if (status != t->len) {
				/* always report some kind of error */
				if (status >= 0)
					status = -EREMOTEIO;
				break;
			}
			status = 0;

			/* protocol tweaks before next transfer */
			if (t->delay_usecs)
				udelay(t->delay_usecs);

			if (!cs_change)
				continue;
			if (t->transfer_list.next == &m->transfers)
				break;

			/* sometimes a short mid-message deselect of the chip
			 * may be needed to terminate a mode or command
			 */
			ndelay(nsecs);
			spi_camelot->chipselect(spi, SPI_CAMELOT_INACTIVE);
			ndelay(nsecs);
		}

		m->status = status;
		m->complete(m->context);

		/* restore speed and wordsize if it was overridden */
		if (do_setup == 1)
			setup_transfer(spi, NULL);
		do_setup = 0;

		/* normally deactivate chipselect ... unless no error and
		 * cs_change has hinted that the next message will probably
		 * be for this chip too.
		 */
		if (!(status == 0 && cs_change)) {
			ndelay(nsecs);
			spi_camelot->chipselect(spi, SPI_CAMELOT_INACTIVE);
			ndelay(nsecs);
		}

		spin_lock_irqsave(&spi_camelot->lock, flags);
	}
	spi_camelot->busy = 0;
	spin_unlock_irqrestore(&spi_camelot->lock, flags);
}

/*
 * spi_camelot_transfer - default submit to transfer queue
 */
int spi_camelot_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct spi_camelot	*spi_camelot;
	unsigned long		flags;
	int			status = 0;

	m->actual_length = 0;
	m->status = -EINPROGRESS;

	spi_camelot = spi_master_get_devdata(spi->master);

	spin_lock_irqsave(&spi_camelot->lock, flags);

	list_add_tail(&m->queue, &spi_camelot->queue);
	queue_work(spi_camelot->workqueue, &spi_camelot->work);

	spin_unlock_irqrestore(&spi_camelot->lock, flags);

	return status;
}

/*----------------------------------------------------------------------*/

static int spi_camelot_bufs(struct spi_device *spi, struct spi_transfer *t)
{
	struct spi_bitbang_cs	*cs = spi->controller_state;

	return cs->txrx_bufs(spi, t);
}

/**
 * spi_camelot_setup - default setup for per-word I/O loops
 */
int spi_camelot_setup(struct spi_device *spi)
{
	int	retval;
	int init_val;
	u32 tmp;
	unsigned long flags;
	struct spi_bitbang_cs *cs = spi->controller_state;
	struct spi_camelot	*spi_camelot;

	spi_camelot = spi_master_get_devdata(spi->master);

	if (!cs) {
		cs = kzalloc(sizeof *cs, GFP_KERNEL);
		if (!cs)
			return -ENOMEM;
		spi->controller_state = cs;
	}

	retval = spi_camelot->setup_transfer(spi, NULL);
	if (retval < 0)
		return retval;

	dev_dbg(&spi->dev, "%s, %u nsec/bit\n", __func__, 2 * cs->nsecs);

	/* NOTE we _need_ to call chipselect() early, ideally with adapter
	 * setup, unless the hardware defaults cooperate to avoid confusion
	 * between normal (active low) and inverted chipselects.
	 */

	/* deselect chip (low or high) */
	spin_lock_irqsave(&spi_camelot->lock, flags);
	if (!spi_camelot->busy) {
		spi_camelot->chipselect(spi, SPI_CAMELOT_INACTIVE);
		ndelay(cs->nsecs);
	}
	spin_unlock_irqrestore(&spi_camelot->lock, flags);

	/* camelot gspi init */
	if(spi->max_speed_hz)
		tmp = min(spi->max_speed_hz, (u32)CML_GSPI_MAX_HZ)<<1;
	else
		tmp = CML_GSPI_MAX_HZ<<1;
	tmp = SYS_CLK%tmp ? (SYS_CLK/tmp)-1 : (SYS_CLK/tmp)-2;

	init_val = tmp<<SPI_CLK_W_OFFSET;
	if(spi->mode & SPI_3WIRE)
		init_val |= SPI_THREE_WIRE;
	if(spi->mode & SPI_CS_HIGH)
		init_val |= SPI_CLK_PARK_HIGH;
	if(spi->mode & SPI_LSB_FIRST)
		init_val |= SPI_DATA_INVERT;
	if(spi->mode & SPI_CPHA)
		init_val |= SPI_FALLING_SAMPLE;

	cml_gspi_init(spi->chip_select,init_val);
	return 0;
}

/**
 * spi_camelot_cleanup - default cleanup for per-word I/O loops
 */
void spi_camelot_cleanup(struct spi_device *spi)
{
	kfree(spi->controller_state);
}

static int __init spi_camelot_probe(struct platform_device *pdev)
{
	int	status;
	struct spi_master *master;
	struct spi_camelot *spi_camelot;
	struct spi_camelot_platform_data *pdata;

	pdata = pdev->dev.platform_data;

	if (!pdata)
		return -ENODEV;

	/* spi master controller driver data structure init */
	master = spi_alloc_master(&pdev->dev, sizeof *spi_camelot);
	cml_gspi_debug("%s: %d master=%x\n", __func__, __LINE__, (int)master);
	if (!master) {
		return -ENOMEM;
	}
	spi_camelot = spi_master_get_devdata(master);
	platform_set_drvdata(pdev, spi_camelot);

	spi_camelot->pdev = pdev;
	if (pdata)
		spi_camelot->pdata = *pdata;

	master->bus_num = pdev->id;
	master->num_chipselect = pdata->num_chipselect;
	master->setup = spi_camelot_setup;
	master->cleanup = spi_camelot_cleanup;

	spi_camelot->master = spi_master_get(master);
	if (!spi_camelot->master)
		return -EINVAL;
	spi_camelot->setup_transfer = spi_camelot_setup_transfer;
	/* supported flags */
	spi_camelot->master->mode_bits = (SPI_CPOL | SPI_CPHA | SPI_3WIRE | SPI_LSB_FIRST | SPI_CS_HIGH);
	spi_camelot->master->transfer = spi_camelot_transfer;
	spi_camelot->txrx_bufs = spi_camelot_bufs;

/////////////////////////////////

	INIT_WORK(&spi_camelot->work, spi_camelot_work);
	spin_lock_init(&spi_camelot->lock);
	INIT_LIST_HEAD(&spi_camelot->queue);

	/* this task is the only thing to touch the SPI bits */
	spi_camelot->busy = 0;
	spi_camelot->chipselect = cml_gspi_chipselect;
	spi_camelot->workqueue = create_singlethread_workqueue(
			dev_name(spi_camelot->master->dev.parent));
	if (spi_camelot->workqueue == NULL) {
		status = -EBUSY;
		goto err1;
	}

	/* driver may get busy before register() returns, especially
	 * if someone registered boardinfo for devices
	 */
	status = spi_register_master(spi_camelot->master);
	if (status < 0)
		goto err2;

	return status;

err2:
	destroy_workqueue(spi_camelot->workqueue);
err1:
	return status;
////////////////////////////////
	
	cml_gspi_debug(" ### %s : %d , status=%d\n", __func__, __LINE__, status);
	if (status < 0) {
		spi_master_put(spi_camelot->master);
	}

	return status;
}

static int __exit spi_camelot_remove(struct platform_device *pdev)
{
	struct spi_camelot			*spi_camelot;
	int				status;

	spi_camelot = platform_get_drvdata(pdev);

	/* stop() unregisters child devices too */
/**
 * spi_bitbang_stop - stops the task providing spi communication
 */
	spi_unregister_master(spi_camelot->master);

	WARN_ON(!list_empty(&spi_camelot->queue));

	destroy_workqueue(spi_camelot->workqueue);
	status =0 ;

	spi_master_put(spi_camelot->master);

	platform_set_drvdata(pdev, NULL);

	return status;
}


struct spi_camelot_platform_data cml_pdata = {
	.num_chipselect = CML_GSPI_CHIPSELECT_NUM,
}; 

static struct platform_device camelot_spi_device = 
{
	.name	= DRIVER_NAME,
	.id		= CML_GSPI_BUS_ID,
	.dev	= {
		.platform_data	= &cml_pdata,
	},
};

static struct platform_driver spi_camelot_driver = {
	.driver.name	= DRIVER_NAME,
	.driver.owner	= THIS_MODULE,
	.remove		= __exit_p(spi_camelot_remove),
	.probe		= spi_camelot_probe,
};

extern struct spi_board_info cml_gspi_devices[]; 
extern int cml_gspi_devices_size;

static int __init spi_camelot_init(void)
{
	int err;
	/* register device */
#define cml_gspi_init_info(...)
	cml_gspi_init_info("%s ===== spi_camelot register platform device =====\n",__func__);
   	err = platform_device_register(&camelot_spi_device);
	cml_gspi_init_info("%s err=%x\n", __func__, err);
	err = platform_driver_register(&spi_camelot_driver);
	/* register board info */
	cml_gspi_init_info("%s ===== spi_camelot register board info =====\n",__func__);
	err= spi_register_board_info(cml_gspi_devices,cml_gspi_devices_size);
	cml_gspi_init_info("if_spi.c : spi register board failed = %d\n",err);
	cml_gspi_init_info("%s ===== spi_camelot register platform driver =====\n",__func__);
	return err;
//	return platform_driver_probe(&spi_camelot_driver, spi_camelot_probe);
}
module_init(spi_camelot_init);

static void __exit spi_camelot_exit(void)
{
	platform_driver_unregister(&spi_camelot_driver);
}
module_exit(spi_camelot_exit);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("AcroSpeed Inc.");
MODULE_DESCRIPTION("Camelot Generic SPI Master Driver");
MODULE_LICENSE("GPL");
