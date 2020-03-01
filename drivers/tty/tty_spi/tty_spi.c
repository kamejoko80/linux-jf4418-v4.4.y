// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * tty_spi.c  -- tty spi driver
 *
 * Reference : ifx6x60.c
 * Written by: Henry Dang
 *
 * --------------------------------------------------------------------------
 *
 * Copyright (c) 2020 Fossil CDG
 * All rights Reserved.
 *
 * --------------------------------------------------------------------------
 */
 
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/sched.h>
#include <linux/serial.h>
#include <linux/interrupt.h>
#include <linux/kmod.h>
#include <linux/init.h>
#include <linux/kfifo.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <asm/byteorder.h>

#define DRIVER_DESC                "tty spi driver"
#define VERSION_STRING DRIVER_DESC " 1.0a"

#define TTY_DRIVER_NAME            "tty_spi" 
#define TTY_NAME                   "ttySPI"
#define TTY_SPI_ID                 (0)
#define TTY_SPI_TRANSFER_SIZE	   (2048)
#define TTY_SPI_FIFO_SIZE		   (4096)
#define TTY_SPI_MODE			   (SPI_MODE_0)

/* IPC define */

#define CPU_MHZ                    (100000)
#define DELAY_UNIT                 (1)
#define HDR_SPI_TRANS_LEN          (4)

#define GPIO_PIN_SET               (1)
#define GPIO_PIN_RESET             (0) 
#define READ_S_STATUS()            (gpio_get_value(priv->status_in))          
#define set_master_busy()          (gpio_set_value(priv->status_out, 1))         
#define set_master_ready()         (gpio_set_value(priv->status_out, 0))      
#define is_slaver_busy()           (gpio_get_value(priv->status_in) == GPIO_PIN_SET)
#define master_send_request()      (gpio_set_value(priv->send_request, 1), gpio_set_value(priv->send_request, 0))   

#define FIXED_SPI_TRANS_SIZE

typedef enum
{
    IPC_STATE_MASTER_INIT                   = 0x00U,
    IPC_STATE_MASTER_SEND_HEADER            = 0x01U,
	IPC_STATE_MASTER_SEND_HEADER_COMPLETED  = 0x02U,
    IPC_STATE_MASTER_REQUEST_HEADER         = 0x03U,
    IPC_STATE_MASTER_HEADER_RECEIVED        = 0x04U,
    IPC_STATE_MASTER_SEND_DATA              = 0x05U,
	IPC_STATE_MASTER_SEND_DATA_COMPLETED    = 0x06U,
	IPC_STATE_MASTER_RECEIVE_DATA           = 0x07U,
	IPC_STATE_MASTER_RECEIVE_DATA_COMPLETED = 0x08U,
	IPC_STATE_MASTER_SYNC                   = 0x09U,
    IPC_STATE_MASTER_ERROR                  = 0x0AU
} ipc_state_master_t;

typedef enum
{
    IPC_STATE_SLAVER_INIT                   = 0x00U,
	IPC_STATE_SLAVER_SEND_HEADER_COMPLETED  = 0x01U,
    IPC_STATE_SLAVER_REQUEST_HEADER         = 0x02U,
    IPC_STATE_SLAVER_HEADER_RECEIVED        = 0x03U,
	IPC_STATE_SLAVER_SEND_DATA_COMPLETED    = 0x04U,
	IPC_STATE_SLAVER_RECEIVE_DATA_COMPLETED = 0x05U,
	IPC_STATE_SLAVER_SYNC                   = 0x06U,
    IPC_STATE_SLAVER_ERROR                  = 0x07U
} ipc_state_slaver_t;

typedef enum
{
	IPC_OK      = 0x00U,
	IPC_ERROR   = 0x01U,
	IPC_BUSY    = 0x02U,
	IPC_TIMEOUT = 0x03U
} ipc_status_t;

typedef struct
{
	u8 *pTxData;
	u8 *pRxData;
	size_t Size;
	ipc_state_master_t State;
	ipc_status_t Status;
} ipc_handle_t;

/* tty spi driver data structure */
struct tty_spi {
	/* tty driver */
	struct tty_port tty_port;
	struct device *tty_dev;
	int minor;

	/* tty port specific data */
	struct kfifo tx_fifo;
	spinlock_t fifo_lock;

	/* spi driver */
	struct spi_device *spi_dev;
	struct spi_transfer spi_transfer;
	struct spi_message spi_msg;
	u8 *spi_tx_buf;
	u8 *spi_rx_buf;
	
	/* module ultilities */
	spinlock_t spin_mutex;
	struct workqueue_struct *wq;
	struct work_struct tty_spi_work;

	/* ctrl gpios */
	int status_in;
	int status_out;
	int send_request;
	int irq_in;
	
	/* gpio irq flag */
	int gpio_irq;
	bool gpio_flag;
};

/* ipc framework local variable */
volatile ipc_handle_t g_IPCHandle;

/* tty driver local variables */
static struct tty_driver *tty_drv;
static struct tty_spi *tty_spi_priv = NULL; 
static struct lock_class_key tty_spi_key;

static const struct of_device_id tty_spi_of_match[] = {
	{
		.compatible	= "fossil,tty_spi",
	},
	{ }
};

MODULE_DEVICE_TABLE(of, tty_spi_of_match);

static const struct spi_device_id tty_spi_id_table[] = {
	{
		.name = "tty_spi",
	},
	{ }
};

/* function prototypes */
static int tty_spi_create_port(struct tty_spi *priv);
static void tty_spi_free_device(struct tty_spi *priv);
static void tty_spi_free_buffers(struct tty_spi *priv);
static void spi_transfer_complete(void *context);
static void spi_write_async(struct tty_spi *priv, const void *buf, size_t len);
static void ipc_master_receive_callback(struct tty_spi *priv);

/* ipc framework implementation */

static void delay_us(u32 us)
{
	u32 i, j;

	for(i = 0; i < (1000000/CPU_MHZ); i++)
	{
		for(j = 0; j < us; j++);
	}
}

static u8 ipc_calculate_checksum(u8 *pData, u16 Size)
{
	u16 i, sum = 0;

	for(i = 0; i < Size; i++)
	{
		sum += pData[i];
	}

	return (0xFF - (sum & 0xFF));
}

static bool ipc_checksum(u8 *pData, u16 Size)
{
	u16 i, sum = 0;

	for(i = 0; i < Size; i++)
	{
		sum += pData[i];
	}

	if((sum & 0xFF) == 0xFF)
	{
		return true;
	}
	else
	{
		return false;
	}
}

static void ipc_build_header(u8 *pHeader, u8 *pData, u16 Size)
{	
	memcpy(pHeader, pData, Size);
	pHeader[Size] = ipc_calculate_checksum(pData, Size);
}

/*
 * ipc master init function
 */
static void ipc_master_init(struct tty_spi *priv)
{
	g_IPCHandle.pRxData = NULL;
	g_IPCHandle.pTxData = NULL;
	g_IPCHandle.Size    = 0;
	g_IPCHandle.Status  = IPC_OK;
	g_IPCHandle.State   = IPC_STATE_MASTER_INIT;

	set_master_ready();
}

/*
 * wait for slaver ready
 */
static bool wait_for_slaver_ready(struct tty_spi *priv, u32 timeout)
{
	u32 to = timeout;

	while(READ_S_STATUS() == GPIO_PIN_SET)
	{
		to--;
		delay_us(DELAY_UNIT);
		if(!to)
		{
			return false;
		}
	}
	return true;
}

static void ipc_master_spi_bus_recovery(struct tty_spi *priv)
{
	ipc_master_init(priv);
}

static void ipc_master_event_handler(struct tty_spi *priv, bool is_gpio_irq)
{
	u8 pHeader[3];
	unsigned long flags;

    /* get SYNC command from the slaver */
	if(is_gpio_irq && is_slaver_busy())
	{
		ipc_master_spi_bus_recovery(priv);
		printk(KERN_ERR "ipc is recovered from remote requested\r\n");
		return;
	}

	switch(g_IPCHandle.State)
	{
		case IPC_STATE_MASTER_INIT:
			if(is_gpio_irq)
			{
				/* slaver initiates to send data */
				g_IPCHandle.State  = IPC_STATE_MASTER_HEADER_RECEIVED;
				g_IPCHandle.Status = IPC_BUSY;

				/* master response state */
				spin_lock_irqsave(&priv->fifo_lock, flags);
				priv->spi_tx_buf[0] = IPC_STATE_MASTER_REQUEST_HEADER;
				spin_unlock_irqrestore(&priv->fifo_lock, flags);

				/* master shifts clock to request header */
				set_master_busy();
				spi_write_async(priv, NULL, HDR_SPI_TRANS_LEN);
			}
			else
			{				
				/* master initiates to send data */
				if(wait_for_slaver_ready(priv, 1000))
				{
					g_IPCHandle.State  = IPC_STATE_MASTER_SEND_HEADER;
					g_IPCHandle.Status = IPC_BUSY;

					master_send_request();
				}
				else
				{
					/* slaver is busy */
					printk(KERN_ERR "Error slaver is busy\r\n");
					g_IPCHandle.Status = IPC_TIMEOUT;
				}
			}
		break;

		case IPC_STATE_MASTER_SEND_HEADER:

			set_master_busy();

			/* master got GPIO IRQ notification from the slaver */
			if(is_gpio_irq)
			{
				g_IPCHandle.State = IPC_STATE_MASTER_SEND_HEADER_COMPLETED;
				ipc_build_header(pHeader, (uint8_t *)&g_IPCHandle.Size, 2);
				spi_write_async(priv, pHeader, HDR_SPI_TRANS_LEN);
			}

		break;

		case IPC_STATE_MASTER_HEADER_RECEIVED:

			/* decode header frame */
			if(ipc_checksum(priv->spi_rx_buf, 3))
			{
				g_IPCHandle.Size = (priv->spi_rx_buf[0] | (priv->spi_rx_buf[1] << 8));
				g_IPCHandle.State = IPC_STATE_MASTER_RECEIVE_DATA;
				//printk(KERN_ERR "g_IPCHandle.Size = %d\r\n", g_IPCHandle.Size);
			}
			else
			{
				g_IPCHandle.State = IPC_STATE_MASTER_ERROR;
				g_IPCHandle.Status = IPC_ERROR;
				printk(KERN_ERR "Error headed checksum\r\n");
			}

			set_master_ready();

		break;

		case IPC_STATE_MASTER_RECEIVE_DATA:
			
			/* master got GPIO IRQ notification from the slaver */
			if(is_gpio_irq)
			{				
				/* master shifts clock to receive data */
				set_master_busy();
				g_IPCHandle.State = IPC_STATE_MASTER_RECEIVE_DATA_COMPLETED;

				/* master response state */
				spin_lock_irqsave(&priv->fifo_lock, flags);
				priv->spi_tx_buf[0] = IPC_STATE_MASTER_HEADER_RECEIVED;
				spin_unlock_irqrestore(&priv->fifo_lock, flags);
				spi_write_async(priv, NULL, g_IPCHandle.Size);
			}

		break;

		case IPC_STATE_MASTER_RECEIVE_DATA_COMPLETED:
			
			g_IPCHandle.State   = IPC_STATE_MASTER_INIT;
			g_IPCHandle.Status  = IPC_OK;
			g_IPCHandle.pRxData = priv->spi_rx_buf;

			ipc_master_receive_callback(priv);

			set_master_ready();

		break;

		case IPC_STATE_MASTER_SEND_HEADER_COMPLETED:
			
			/* check slaver response state */
			if(priv->spi_rx_buf[0] == IPC_STATE_SLAVER_REQUEST_HEADER)
			{
				g_IPCHandle.State = IPC_STATE_MASTER_SEND_DATA;
				set_master_ready();
			}
			else
			{
				printk(KERN_ERR "Error slaver request header state %x\r\n", priv->spi_rx_buf[0]);
				g_IPCHandle.State  = IPC_STATE_MASTER_ERROR;
				g_IPCHandle.Status = IPC_ERROR;
				set_master_busy();
			}

		break;

		case IPC_STATE_MASTER_SEND_DATA:
	
			set_master_busy();

			/* master got GPIO IRQ notification from the slaver */
			if(is_gpio_irq)
			{
				g_IPCHandle.State = IPC_STATE_MASTER_SEND_DATA_COMPLETED;
				spi_write_async(priv, g_IPCHandle.pTxData, g_IPCHandle.Size);
			}

		break;

		case IPC_STATE_MASTER_SEND_DATA_COMPLETED:
			
			/* check slaver response state */
			if(priv->spi_rx_buf[0] == IPC_STATE_SLAVER_HEADER_RECEIVED)
			{
				g_IPCHandle.State  = IPC_STATE_MASTER_INIT;
				g_IPCHandle.Status = IPC_OK;
				set_master_ready();
			}
			else
			{
				printk(KERN_ERR "Error slaver header received state %X\r\n", priv->spi_rx_buf[0]);
				g_IPCHandle.State  = IPC_STATE_MASTER_ERROR;
				g_IPCHandle.Status = IPC_ERROR;
				set_master_busy();
			}

		break;

		case IPC_STATE_MASTER_ERROR:
				g_IPCHandle.Status = IPC_ERROR;
				set_master_busy();
		break;

		case IPC_STATE_MASTER_SYNC:
		    /*
		     * master recovers itself SPI bus and
			 * send SYNC signal to the slaver
			 */ 
			ipc_master_spi_bus_recovery(priv);
			set_master_busy();
			master_send_request();
			delay_us(400);
			set_master_ready();
			printk(KERN_ERR "ipc is recovered from sync command\r\n");
		break;

		default:
			g_IPCHandle.State  = IPC_STATE_MASTER_ERROR;
			g_IPCHandle.Status = IPC_ERROR;
			set_master_busy();
		break;
	}
}

/*
 * master recovers and send sync command to the slaver
 */
static void ipc_master_sync_request(struct tty_spi *priv)
{
	g_IPCHandle.State = IPC_STATE_MASTER_SYNC;
	ipc_master_event_handler(priv, false);
}

/*
 * ipc master send
 */
static ipc_status_t ipc_master_send(struct tty_spi *priv, u8 *pTxData, u16 Size)
{
	u32 timeout = 1000;
	
	if(Size > TTY_SPI_TRANSFER_SIZE)
	{
		printk(KERN_ERR "Error! data size exceeds max allowance\r\n");
		return IPC_ERROR;
	}

	if(g_IPCHandle.Status != IPC_OK)
	{
		printk(KERN_ERR "Error! previous operation, need to recovery the driver\r\n");
		return IPC_ERROR;
	}

	/* Initialize IPC handle info */
	g_IPCHandle.pTxData = pTxData;
	g_IPCHandle.Size = Size;	

	/* master initiate send data */
	ipc_master_event_handler(priv, false);

	if(g_IPCHandle.Status == IPC_TIMEOUT)
	{
		return IPC_TIMEOUT;
	}

	/* wait for master transfer complete */
	while(g_IPCHandle.Status != IPC_OK) {
		
		timeout--;

		if(!timeout) {
			printk(KERN_ERR "ipc_master_send timeout\r\n");
			return IPC_TIMEOUT;
		}
		
		mdelay(1);
		
		if(g_IPCHandle.Status == IPC_ERROR){
			printk(KERN_ERR "ipc_master_send error\r\n");
			return IPC_ERROR;
		}
	}

	printk(KERN_ERR "ipc_master_send completed\r\n");

	return IPC_OK;
}

static void ipc_master_receive_callback(struct tty_spi *priv)
{	
	printk(KERN_ERR "===> ipc_master_receive_callback\r\n");
	printk(KERN_ERR "%s\r\n", g_IPCHandle.pRxData);
}

/* spi device driver implementation */

static void spi_write_async(struct tty_spi *priv, const void *buf, size_t len)
{
	struct device *dev = (struct device *)&priv->spi_dev->dev;
	unsigned long flags;
	int err;

	//printk(KERN_ERR "===> spi_write_async\r\n");

	if(buf != NULL) {
		/* clear spi tx buffer */
		spin_lock_irqsave(&priv->fifo_lock, flags);
		memset((void *)priv->spi_tx_buf, 0, TTY_SPI_TRANSFER_SIZE);
		memcpy((void *)priv->spi_tx_buf, buf, len);
		spin_unlock_irqrestore(&priv->fifo_lock, flags);
	}

	/* init spi transfer */
	priv->spi_transfer.tx_buf = priv->spi_tx_buf; 
	priv->spi_transfer.rx_buf = priv->spi_rx_buf;

#ifdef FIXED_SPI_TRANS_SIZE
	if((len % 4) != 0){
		priv->spi_transfer.len = len - (len % 4) + 4;
	} else {
		priv->spi_transfer.len = len;
	}
#else
	priv->spi_transfer.len = len;
#endif

	/* init spi message */
	spi_message_init(&priv->spi_msg);

	/* setup spi transfer complete */
	priv->spi_msg.complete = &spi_transfer_complete;
	priv->spi_msg.context = (void *)priv;

	/* add spi transfer to the list */
	spi_message_add_tail(&priv->spi_transfer, &priv->spi_msg);	

	/* queue spi transfer */
	err = spi_async(priv->spi_dev, &priv->spi_msg);
	if (err) {
		dev_err(dev, "spi queue failed: err = %d\n", err);
	}
}

static void tty_spi_work_handler(struct work_struct *ws)
{
	struct tty_spi *priv = container_of(ws, struct tty_spi, tty_spi_work);
	
	//printk(KERN_ERR "===> priv->gpio_flag = %d\r\n", priv->gpio_flag);
	
	ipc_master_event_handler(priv, priv->gpio_flag);
}

static void spi_transfer_complete(void *context)
{
	struct tty_spi *priv = (struct tty_spi *)context;
	
	//printk(KERN_ERR "===> spi_transfer_complete\r\n");
	
	/* notify there is incomming data */
	priv->gpio_flag = false;
	queue_work(priv->wq, &priv->tty_spi_work);
}

static irqreturn_t tty_spi_irq_in_isr(int irq, void *devid)
{
	struct tty_spi *priv = (struct tty_spi *)devid;
	
	//printk(KERN_ERR "===> tty_spi_irq_in_isr\r\n");
	
	priv->gpio_flag = true;

	queue_work(priv->wq, &priv->tty_spi_work);

	return IRQ_HANDLED;
}

static int tty_spi_gpio_init(struct tty_spi *priv)
{
	struct device *dev = (struct device *)&priv->spi_dev->dev;
	int ret = 0;
	
	ret = gpio_request(priv->status_out, "status_out");
	if (ret) {
		dev_err(dev, "gpio is busy\r\n");
		return ret;
	}

	ret = gpio_direction_output(priv->status_out, 1);
	if (ret) {
		dev_err(dev, "gpio set ouput failed\r\n");
		return ret;
	}

	ret = gpio_request(priv->send_request, "send_request");
	if (ret) {
		dev_err(dev, "gpio is busy\r\n");
		return ret;
	}

	ret = gpio_direction_output(priv->send_request, 0);
	if (ret) {
		dev_err(dev, "gpio set ouput failed\r\n");
		return ret;
	}
	
	ret = gpio_request(priv->status_in, "status_in");
	if (ret) {
		dev_err(dev, "gpio is busy\r\n");
		return ret;
	}

	ret = gpio_direction_input(priv->status_in);
	if (ret) {
		dev_err(dev, "gpio set ouput failed\r\n");
		return ret;
	}
	
	return ret;
}

static void tty_spi_gpio_deinit(struct tty_spi *priv)
{
	gpio_free(priv->status_out);
	gpio_free(priv->send_request);
	gpio_free(priv->status_in);	
}

static int tty_spi_probe(struct spi_device *spi)
{
	const struct of_device_id *match;
	struct tty_spi *priv;
	int ret = 0;

	if (tty_spi_priv) {
		dev_dbg(&spi->dev, "ignoring subsequent detection");
		return -ENODEV;
	}

	priv = kzalloc(sizeof(struct tty_spi), GFP_KERNEL);

	if (!priv) {
		ret = -ENOMEM;
		dev_err(&spi->dev, "could not allocate memory\r\n");
		goto err_exit;
	}
	
	/* save driver data */
	tty_spi_priv = priv;
	priv->spi_dev = spi;
	
	/* allocate spi tx/rx buffers */
	priv->spi_tx_buf = devm_kzalloc(&spi->dev, TTY_SPI_TRANSFER_SIZE, GFP_KERNEL);

	if (!priv->spi_tx_buf) {
		ret = -ENOMEM;
		goto free_priv;
	}

	priv->spi_rx_buf = devm_kzalloc(&spi->dev, TTY_SPI_TRANSFER_SIZE, GFP_KERNEL);

	if (!priv->spi_rx_buf) {
		ret = -ENOMEM;
		goto free_priv;
	}

	/* initialize mutex lock */
	spin_lock_init(&priv->spin_mutex);

	/* get gpio pin number from device tree */
	match = of_match_device(of_match_ptr(tty_spi_of_match), &spi->dev);

	if(match) {
		priv->status_in = of_get_named_gpio(spi->dev.of_node, "status_in", 0);
		priv->status_out = of_get_named_gpio(spi->dev.of_node, "status_out", 0);
		priv->send_request = of_get_named_gpio(spi->dev.of_node, "send_request", 0);
		priv->irq_in = of_get_named_gpio(spi->dev.of_node, "irq_in", 0);		
	}	

	/* spi mode setting */
	spi->mode = TTY_SPI_MODE;
	spi->bits_per_word = 8;

	/* setup spi */
	ret = spi_setup(spi);
	if (ret) {
		dev_err(&spi->dev, "setup spi failed\r\n");
		goto free_priv;
	}

	/* init spi transfer */
	priv->spi_transfer.tx_buf = priv->spi_tx_buf; 
	priv->spi_transfer.rx_buf = priv->spi_rx_buf;
	priv->spi_transfer.len = TTY_SPI_TRANSFER_SIZE;
	priv->spi_transfer.cs_change = 0;

	/* store driver data */
	spi_set_drvdata(spi, priv);

	/* init spi message */
	spi_message_init(&priv->spi_msg);
	
	/* init tty spi workqueue */
	priv->wq = create_freezable_workqueue("tty_spi_work_queue");
	INIT_WORK(&priv->tty_spi_work, tty_spi_work_handler);

	/* setup gpio */
	ret = tty_spi_gpio_init(priv);
	if (ret) {
		goto free_priv;
	}

	/* register gpio interrupt */
	ret = request_threaded_irq(gpio_to_irq(priv->irq_in), 
							NULL, 
							tty_spi_irq_in_isr,
							IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
							spi->dev.driver->name, priv);
	if (ret) {
		dev_err(&spi->dev, "could not regster irq %d\r\n", gpio_to_irq(priv->irq_in));
		goto free_priv;
	}

	/* store gpio irq */
	priv->gpio_irq = gpio_to_irq(priv->irq_in);
	
	/* we will enable it later */
	disable_irq_nosync(priv->gpio_irq);

    /* create our tty port */
    ret = tty_spi_create_port(priv);
    if (ret) {
        dev_err(&spi->dev, "create default tty port failed\r\n");
        goto free_priv;
    }

	printk(KERN_ERR "%s successfully\n", __func__);
	
	return 0;

free_priv:
	tty_spi_free_buffers(priv);
	kfree(priv);
	tty_spi_priv = NULL;

err_exit:
	return ret;
}

static void tty_spi_shutdown(struct spi_device *spi)
{

}

static int tty_spi_remove(struct spi_device *spi)
{
	struct tty_spi *priv = spi_get_drvdata(spi);

	/* free irq */
	free_irq(gpio_to_irq(priv->irq_in), priv);

	/* free gpios */
	tty_spi_gpio_deinit(priv);

	/* free the workqueue */
	destroy_workqueue(priv->wq);

	/* free tty spi device */
	tty_spi_free_device(priv);

	if (priv) {
		kfree(priv);
		tty_spi_priv = NULL;
	}
	
	return 0;
}

static int tty_spi_pm_resume(struct device *dev)
{
    return 0;
}

static int tty_spi_pm_suspend(struct device *dev)
{
    return 0;
}

static int tty_spi_pm_runtime_resume(struct device *dev)
{
    return 0;
}

static int tty_spi_pm_runtime_suspend(struct device *dev)
{
    return 0;
}

static int tty_spi_pm_runtime_idle(struct device *dev)
{
    return 0;
}

static const struct dev_pm_ops tty_spi_pm_ops = {
    .resume = tty_spi_pm_resume,
    .suspend = tty_spi_pm_suspend,
    .runtime_resume = tty_spi_pm_runtime_resume,
    .runtime_suspend = tty_spi_pm_runtime_suspend,
    .runtime_idle = tty_spi_pm_runtime_idle
};

static struct spi_driver tty_spi_driver = {
	.driver = {
		.name = TTY_DRIVER_NAME,
		.of_match_table = tty_spi_of_match,
		.pm = &tty_spi_pm_ops,
	},
	.id_table = tty_spi_id_table,
	.probe = tty_spi_probe,
	.shutdown = tty_spi_shutdown,
	.remove = tty_spi_remove,
};

/* tty device driver implementation */

static int tty_spi_port_activate(struct tty_port *port, struct tty_struct *tty)
{	
	struct tty_spi *priv = container_of(port, struct tty_spi, tty_port);
	
	/* reset fifo */
	kfifo_reset(&priv->tx_fifo);
	
	/* save driver data */
	tty->driver_data = priv;

	/* allows flip string push from int context */
	port->low_latency = 1;	
	
	return 0;
}

static void tty_spi_port_shutdown(struct tty_port *port)
{
	// struct tty_spi *priv = container_of(port, struct tty_spi, tty_port);
	
	// clear_bit(IFX_SPI_STATE_IO_AVAILABLE, &ifx_dev->flags);
	// mrdy_set_low(ifx_dev);
	// del_timer(&ifx_dev->spi_timer);
	// clear_bit(IFX_SPI_STATE_TIMER_PENDING, &ifx_dev->flags);
	// tasklet_kill(&ifx_dev->io_work_tasklet);	
}

static const struct tty_port_operations tty_spi_port_ops = {
    .activate = tty_spi_port_activate,
    .shutdown = tty_spi_port_shutdown,
};

static void tty_spi_free_port(struct tty_spi *priv)
{
    if (priv->tty_dev)
        tty_unregister_device(tty_drv, priv->minor);
	tty_port_destroy(&priv->tty_port);
	kfifo_free(&priv->tx_fifo);
}

static void tty_spi_free_device(struct tty_spi *priv)
{
	tty_spi_free_port(priv);
	
	if(priv->spi_tx_buf) {
		kfree(priv->spi_tx_buf);
		priv->spi_tx_buf = NULL;
	}

	if(priv->spi_rx_buf) {
		kfree(priv->spi_rx_buf);
		priv->spi_rx_buf = NULL;
	}
}

static void tty_spi_free_buffers(struct tty_spi *priv)
{
	if(priv->spi_tx_buf) {
		kfree(priv->spi_tx_buf);
		priv->spi_tx_buf = NULL;
	}

	if(priv->spi_rx_buf) {
		kfree(priv->spi_rx_buf);
		priv->spi_rx_buf = NULL;
	}	
}

static int tty_spi_create_port(struct tty_spi *priv)
{
	struct tty_port *pport = &priv->tty_port;
	int ret = 0;

	spin_lock_init(&priv->fifo_lock);
	lockdep_set_class_and_subclass(&priv->fifo_lock, &tty_spi_key, 0);

	if (kfifo_alloc(&priv->tx_fifo, TTY_SPI_FIFO_SIZE, GFP_KERNEL)) {
		ret = -ENOMEM;
		goto error_ret;
	}

    tty_port_init(pport);
    pport->ops = &tty_spi_port_ops;
	priv->minor = TTY_SPI_ID;
	
    priv->tty_dev = tty_port_register_device(pport, tty_drv,
            priv->minor, &priv->spi_dev->dev);
			
    if (IS_ERR(priv->tty_dev)) {
        dev_err(&priv->spi_dev->dev,
            "%s: registering tty device failed", __func__);
        ret = PTR_ERR(priv->tty_dev);
        goto error_port;
    }

	printk(KERN_ERR "===> tty_spi_create_port successfully\r\n");

    return 0;

error_port:
	tty_port_destroy(pport);
error_ret:
    tty_spi_free_port(priv);

    return ret;
}

static int tty_spi_open(struct tty_struct *tty, struct file *filp)
{	
	int ret;
	
	//printk(KERN_ERR "===> %s\r\n", __func__);

	ret = tty_port_open(&tty_spi_priv->tty_port, tty, filp);
	
	if(ret){
		printk(KERN_ERR "===> %s failed\r\n", __func__);
	}
	
	/* get driver data */
	struct tty_spi *priv = tty->driver_data;
	
	/* enable gpio irq */
	enable_irq(priv->gpio_irq);
	
	/* send sync command to the slaver */
	ipc_master_sync_request(priv);
	
#if 0	
	/* Test SPI DMA transfer */
	char test[] = {
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
		0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55
	};	

	printk(KERN_ERR "===> ipc_master_send\r\n");
	ipc_master_send(priv, test, 32);
#endif
	
	return ret;
}

static void tty_spi_close(struct tty_struct *tty, struct file *filp)
{
	struct tty_spi *priv = tty->driver_data;
	
	//printk(KERN_ERR "===> %s\r\n", __func__);	
	
	/* disable gpio irq */
	//disable_irq_nosync(priv->gpio_irq);
	
	if(priv){
		tty_port_close(&priv->tty_port, tty, filp);
	} else {
		printk(KERN_ERR "===> warning %s: driver data is not saved\r\n", __func__);
	}
}

static int tty_spi_write(struct tty_struct *tty, const unsigned char *buffer,
						int count)
{	
	struct tty_spi *priv = tty->driver_data;
	unsigned char *tmp_buf = (unsigned char *)buffer;
	unsigned long flags;
	bool is_fifo_empty;
	int tx_count;

	printk(KERN_ERR "===> %s : %s\r\n", __func__, buffer);

	spin_lock_irqsave(&priv->fifo_lock, flags);
	is_fifo_empty = kfifo_is_empty(&priv->tx_fifo);
	tx_count = kfifo_in(&priv->tx_fifo, tmp_buf, count);
	spin_unlock_irqrestore(&priv->fifo_lock, flags);
	
	/* Test SPI DMA transfer */
	// spi_write_async(priv, buffer, count);
	
	//if (is_fifo_empty)
	//	mrdy_assert(ifx_dev);

	return tx_count;	
	
}

static void tty_spi_hangup(struct tty_struct *tty)
{
	struct tty_spi *priv = tty->driver_data;
	printk(KERN_ERR "===> %s\r\n", __func__);
	tty_port_hangup(&priv->tty_port);	
}

static int tty_spi_write_room(struct tty_struct *tty)
{
	struct tty_spi *priv = tty->driver_data;
	printk(KERN_ERR "===> %s\r\n", __func__);
	return TTY_SPI_FIFO_SIZE - kfifo_len(&priv->tx_fifo);	
}

static s32 tty_spi_chars_in_buffer(struct tty_struct *tty)
{	
	struct tty_spi *priv = tty->driver_data;	
	//printk(KERN_ERR "===> %s\r\n", __func__);
	return kfifo_len(&priv->tx_fifo);
}

static int tty_spi_tiocmget(struct tty_struct *tty)
{
	printk(KERN_ERR "===> %s\r\n", __func__);	
	return 0;
}

static int tty_spi_tiocmset(struct tty_struct *tty, unsigned int set, unsigned int clear)
{
	printk(KERN_ERR "===> %s\r\n", __func__);	
	return 0;
}

static const struct tty_operations tty_spi_ops = {
	.open = tty_spi_open,
	.close = tty_spi_close,
	.write = tty_spi_write,
	.hangup = tty_spi_hangup,
	.write_room = tty_spi_write_room,
	.chars_in_buffer = tty_spi_chars_in_buffer,
	.tiocmget = tty_spi_tiocmget,
	.tiocmset = tty_spi_tiocmset,
};

static __init int tty_spi_init(void)
{
	int ret;

	printk(KERN_INFO "Initializing %s\n", VERSION_STRING);

	tty_drv = alloc_tty_driver(1);
	if (!tty_drv)
		return -ENOMEM;

	tty_drv->driver_name = TTY_DRIVER_NAME;
	tty_drv->name = TTY_NAME;
	tty_drv->minor_start = TTY_SPI_ID;
	tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
	tty_drv->subtype = SERIAL_TYPE_NORMAL;
	tty_drv->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_drv->init_termios = tty_std_termios;
	
	/* set operation */
	tty_set_operations(tty_drv, &tty_spi_ops);

	ret = tty_register_driver(tty_drv);
	if (ret) {
		printk(KERN_ERR "tty_spi: failed to register tty driver\n");
		goto free_tty;
	}

	ret = spi_register_driver((void *)&tty_spi_driver);
	if (ret) {
		printk(KERN_ERR "%s: spi_register_driver failed(%d)",
				TTY_DRIVER_NAME, ret);
		goto unreg_tty;
	}
	else
	{
		printk(KERN_ERR "register %s successfully\n", DRIVER_DESC);
	}

	return 0;

unreg_tty:
    tty_unregister_driver(tty_drv);
free_tty:
	put_tty_driver(tty_drv);
	return ret;
}

static __exit void tty_spi_exit(void)
{
	printk(KERN_INFO "Unloading %s\n", DRIVER_DESC);
	spi_unregister_driver((void *)&tty_spi_driver);
	tty_unregister_driver(tty_drv);
	put_tty_driver(tty_drv);
	tty_drv = NULL;
}

module_init(tty_spi_init);
module_exit(tty_spi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Henry Dang");
MODULE_DESCRIPTION(DRIVER_DESC);
