/*
 * SPI emulated ethernet driver
 *
 * Copyright (C) 2020 Henry Dang
 * Author: Henry Dang 
 * based on enc28j60.c written by Claudio Lanconelli for 4.x kernel version
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kfifo.h>
#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/tcp.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

	/** SPI GPIO Configuration (SPI master)
    Pin 24  ------> SPI0_CSN
    Pin 23  ------> SPI1_SCK
    Pin 21  ------> SPI1_MISO
    pin 19  ------> SPI1_MOSI

    Pin 07  ------> m_status_out
    Pin 15  ------> s_status_in
    Pin 11  ------> m_send_rqst
    Pin 13  ------> m_irq_in
    */

	/** SPI GPIO Configuration (SPI slaver)
    PA15     ------> SPI3_NSS
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI

    PB8      ------> s_status_out
    PB9      ------> m_status_in
    PE0      ------> s_send_rqst (need to pull down by external resistor)
    PB1      ------> s_irq_in
    */

	/** Connection
    Master  <----->  Slaver

    Pin 24  ------> PA15 (SPI_NSS)
    Pin 23  ------> PC10 (SPI_SCK)
    Pin 21  <-----  PC11 (SPI_MISO)
    Pin 19  ------> PC12 (SPI_MOSI)

    Pin 07  ------> PB9  (m_status_out ---> m_status_in)
    Pin 15  <-----  PB8  (s_status_out ---> s_status_in)
    Pin 13  <-----  PE0  (s_send_rqst  ---> m_irq_in)
    Pin 11  ------> PB1  (s_irq_in     ---> m_send_rqst)
    */

#define DRV_NAME	"spinet"
#define DRV_VERSION	"1.01"

#define SPI_MODE			    (SPI_MODE_0)

#define SPINET_MSG_DEFAULT	\
	(NETIF_MSG_PROBE | NETIF_MSG_IFUP | NETIF_MSG_IFDOWN | NETIF_MSG_LINK)

#define TX_TIMEOUT	(4 * HZ)

/* IPC definitcation */
#define GPIO_PIN_SET               (1)
#define GPIO_PIN_RESET             (0) 
#define READ_S_STATUS()            (gpio_get_value(priv->status_in))          
#define set_master_busy()          (gpio_set_value(priv->status_out, 1))         
#define set_master_ready()         (gpio_set_value(priv->status_out, 0))      
#define is_slaver_busy()           (gpio_get_value(priv->status_in) == GPIO_PIN_SET)
#define master_send_request()      (gpio_set_value(priv->send_request, 1), gpio_set_value(priv->send_request, 0))  

#define CPU_MHZ                    (100000)
#define DELAY_UNIT                 (1)
#define WAITTIME     			   (200)

/*!
 * IPC parameter configuration
 */
#define IPC_TRANSFER_LEN (1524)
#define IPC_DATA_MAX_LEN (IPC_TRANSFER_LEN - 5)
#define IPC_FRAME_SOH    (0xA5)

#define FIFO_DEPTH       (10)
#define FIFO_BUF_SIZE    (FIFO_DEPTH*IPC_TRANSFER_LEN)

/*
 * IPC Frame structure:
 *  _____ _____ ______ ______ ____________ ______________
 * |     |     |      |      |            |              |
 * | SOH | LEN | CRC8 | CRC8 |    DATA    | ZERO PADDING |
 * |_____|_____|______|______|____________|______________|
 *
 * |<----------------- IPC_TRANSFER_LEN ---------------->|
 *
 *  SOH  : uint8_t  : Start of header
 *  LEN  : uint16_t : Data length (not include data CRC8)
 *  CRC8 : uint8_t  : Header CRC checksum
 *  CRC8 : uint8_t  : Frame data checksum
 *  DATA : uint8_t  : IPC frame data
 *
 */

/* IPC frame header structure */
typedef struct
{
    uint8_t  soh;    /* Start of header     */
    uint16_t len;    /* Payload length      */
    uint8_t  crc8;   /* Header CRC checksum */
} __attribute__((packed, aligned(1))) ipc_frame_header_t;

/*!
 * IPC frame structure
 */
typedef struct
{
	ipc_frame_header_t header;      /* Frame header      */
    uint8_t crc8;                   /* Data CRC checksum */
    uint8_t data[IPC_DATA_MAX_LEN]; /* Frame data        */
} __attribute__((packed, aligned(1))) ipc_frame_t;

typedef enum
{
	IPC_OK         = 0x00U,
	IPC_BUSY       = 0x01U,
	IPC_ERR        = 0x02U
} ipc_status_t;

/* IPC transfer complete status */
static bool ipc_transfer_complete = true;

/* Driver local data */
struct spinet {
	
	/* net dev */
	struct net_device *netdev;
	
	/* spi dev */
	struct spi_device *spi;
	struct spi_transfer spi_transfer;
	struct spi_message spi_msg;
	
	/* spi transfer buffers */
	u8 *tx_buff;
	u8 *rx_buff;
	
	/* ipc transmit fifo buffer */
	struct kfifo ipc_tx_fifo;
	
	/* resource lock */
	spinlock_t buff_lock;
	struct mutex lock;
	
	/* skb buffer */
	struct sk_buff *tx_skb;
	
	/* scheduler */
	struct work_struct spi_work;
	struct work_struct tx_work;
	struct work_struct irq_work;
	struct work_struct setrx_work;
	struct work_struct restart_work;
	
	u8 bank;		/* current register bank selected */
	u16 next_pk_ptr;	/* next packet pointer within FIFO */
	u16 max_pk_counter;	/* statistics: max packet counter */
	u16 tx_retry_count;
	bool hw_enable;
	bool full_duplex;
	int rxfilter;
	u32 msg_enable;	
	//u8 spi_transfer_buf[SPI_TRANSFER_BUF_LEN];

	/* ctrl gpios */
	int status_in;
	int status_out;
	int send_request;
	int irq_in;
	
	/* gpio irq flag */
	int gpio_irq;
	bool gpio_flag;
};

/* use ethtool to change the level for any given device */
static struct {
	u32 msg_enable;
} debug = { 1 };

static unsigned long msec20_to_jiffies;

static const struct of_device_id spinet_of_match[] = {
	{
		.compatible	= "fossil,spinet",
	},
	{ }
};

MODULE_DEVICE_TABLE(of, spinet_of_match);

static const struct spi_device_id spinet_id_table[] = {
	{
		.name = "spinet",
	},
	{ }
};

/* IPC implementation */

static void spi_write_async(struct spinet *priv, u8 *buf);


static void delay_us(u32 us)
{
	u32 i, j;

	for(i = 0; i < (1000000/CPU_MHZ); i++){
		for(j = 0; j < us; j++);
	}
}

static u8 ipc_frame_crc8(u8 *data, size_t len)
{
  unsigned crc = 0;
  int i, j;

  /* Using x^8 + x^2 + x + 1 polynomial */
  for (j = len; j; j--, data++) {
    crc ^= (*data << 8);

    for(i = 8; i; i--) {
      if (crc & 0x8000) {
        crc ^= (0x1070 << 3);
      }
      crc <<= 1;
    }
  }
  return (u8)(crc >> 8);
}

static bool ipc_frame_create(struct spinet *priv, ipc_frame_t *frame, u8 *data, size_t len)
{
    bool ret = false;
	unsigned long flags;

    if(len <= IPC_DATA_MAX_LEN) {

		spin_lock_irqsave(&priv->buff_lock, flags);
		
        /* Reset IPC frame */
        memset((void *)frame, 0, sizeof(ipc_frame_t));

        /* Creates IPC frame header */
        frame->header.soh  = IPC_FRAME_SOH;
        frame->header.len  = len;
        frame->header.crc8 = ipc_frame_crc8((u8 *)&frame->header, 3);

        /* Create IPC frame data and CRC8 */
        memcpy(frame->data, data, len);
        frame->crc8 = ipc_frame_crc8(frame->data, len);
		
        ret = true;
				
		spin_unlock_irqrestore(&priv->buff_lock, flags);

    } else {
        printk(KERN_ERR "Error data length exceeds allow max length\r\n");
    }

    return ret;
}

static bool ipc_frame_check(ipc_frame_t *frame)
{
    bool ret = false;

    if(frame->header.soh != IPC_FRAME_SOH) {
        //printk(KERN_ERR "Error IPC frame header SOH\r\n");
    } else if(ipc_frame_crc8((u8 *)&frame->header, 3) != frame->header.crc8) {
        //printk(KERN_ERR "Error IPC frame header\r\n");
    } else if(ipc_frame_crc8(frame->data, frame->header.len) != frame->crc8) {
        //printk(KERN_ERR "Error IPC frame data\r\n");
    } else {
        ret = true;
    }

    return ret;
}

static void ipc_frame_print(ipc_frame_t *frame)
{
    int i;

    printk(KERN_ERR "===== IPC Frame Info =====\r\n");
    printk(KERN_ERR "Header:\r\n");
    printk(KERN_ERR "  Soh : 0x%X\r\n", frame->header.soh);
    printk(KERN_ERR "  Len : %d\r\n", frame->header.len);
    printk(KERN_ERR "  Data: ");

    for (i = 0; i < frame->header.len; i++) {
        printk(KERN_ERR "%X ", frame->data[i]);
    }

    printk(KERN_ERR "\r\n");
}

static void ipc_init(struct spinet *priv)
{
	set_master_ready();
	ipc_transfer_complete = true;
	enable_irq(priv->gpio_irq);
}

static ipc_status_t ipc_send(struct spinet *priv, u8 *data, size_t len)
{
    ipc_frame_t *frame = (ipc_frame_t *)priv->tx_buff;
    ipc_status_t ret = IPC_OK;

    if(!ipc_transfer_complete) {
    	return IPC_BUSY;
    }

    if(!is_slaver_busy()) {
        if(ipc_frame_create(priv, frame, data, len)) {
            set_master_busy();
            master_send_request();
            mdelay(WAITTIME);
            ipc_transfer_complete = false;
			spi_write_async(priv, (u8 *)frame);
        	ret = IPC_OK;
        } else {
			printk(KERN_ERR "===== ipc frame create error =====\r\n");
        	ret = IPC_ERR;
        }
    } else {
		printk(KERN_ERR "===== is_slaver_busy =====\r\n");
       	ret = IPC_BUSY;
    }

    return ret;
}

static void sync_command(struct spinet *priv)
{
	ipc_init(priv);
	master_send_request();
	delay_us(WAITTIME);
}

static void ipc_receive_callback(struct spinet *priv)
{
	ipc_frame_t *frame = (ipc_frame_t *)priv->rx_buff;

	if(ipc_frame_check(frame))
	{
		ipc_frame_print(frame);
	}
}

static void spinet_spi_work_handler(struct work_struct *work)
{
	struct spinet *priv = container_of(work, struct spinet, spi_work);

	//printk(KERN_ERR "===> spinet_spi_work_handler\r\n");

	ipc_receive_callback(priv);
	set_master_ready();
	ipc_transfer_complete = true;
}

static void spinet_irq_work_handler(struct work_struct *work)
{
	struct spinet *priv = container_of(work, struct spinet, irq_work);

	//printk(KERN_ERR "===> spinet_irq_work_handler\r\n");	
	
	if(!is_slaver_busy())
	{
		printk(KERN_ERR "===> sync cmd\r\n");
		ipc_init(priv);
	}
	else if(ipc_transfer_complete)
	{
		set_master_busy();
		ipc_transfer_complete = false;
		spi_write_async(priv, NULL);
	}	
}

static irqreturn_t spinet_gpio_irq(int irq, void *devid)
{
	struct spinet *priv = (struct spinet *)devid;

    //printk(KERN_ERR "===> spinet_gpio_irq\r\n");

	schedule_work(&priv->irq_work);

	return IRQ_HANDLED;
}

static void spi_transfer_complete(void *context)
{
	struct spinet *priv = (struct spinet *)context;

	//printk(KERN_ERR "===> spi_transfer_complete\r\n");
	
	schedule_work(&priv->spi_work);
}

static void spi_write_async(struct spinet *priv, u8 *buf)
{
	struct device *dev = (struct device *)&priv->spi->dev;
	unsigned long flags;
	int err;

	//printk(KERN_ERR "===> spi_write_async\r\n");

	if(buf == NULL) {
		spin_lock_irqsave(&priv->buff_lock, flags);
		memset((void *)priv->tx_buff, 0, IPC_TRANSFER_LEN);
		spin_unlock_irqrestore(&priv->buff_lock, flags);		
	}

	/* init spi transfer */
	priv->spi_transfer.tx_buf = priv->tx_buff; 
	priv->spi_transfer.rx_buf = priv->rx_buff;
	priv->spi_transfer.len = IPC_TRANSFER_LEN;

	/* init spi message */
	spi_message_init(&priv->spi_msg);

	/* setup spi transfer complete */
	priv->spi_msg.complete = &spi_transfer_complete;
	priv->spi_msg.context = (void *)priv;

	/* add spi transfer to the list */
	spi_message_add_tail(&priv->spi_transfer, &priv->spi_msg);	

	/* queue spi transfer */
	err = spi_async(priv->spi, &priv->spi_msg);
	if (err) {
		dev_err(dev, "spi queue failed: err = %d\n", err);
	}
}

/* spinet function description */

static void dump_packet(const char *msg, int len, const char *data)
{
	printk(KERN_ERR DRV_NAME ": %s - packet len:%d\n", msg, len);
	print_hex_dump(KERN_ERR, "pk data: ", DUMP_PREFIX_OFFSET, 16, 1,
			data, len, true);
}

static int spinet_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct spinet *priv = netdev_priv(dev);

	printk(KERN_ERR "===> %s\r\n", __func__);

	cmd->transceiver = XCVR_INTERNAL;
	cmd->supported	= SUPPORTED_10baseT_Half
			| SUPPORTED_10baseT_Full
			| SUPPORTED_TP;
	ethtool_cmd_speed_set(cmd,  SPEED_10);
	cmd->duplex	= priv->full_duplex ? DUPLEX_FULL : DUPLEX_HALF;
	cmd->port	= PORT_TP;
	cmd->autoneg = AUTONEG_DISABLE;

	return 0;
}

static int spinet_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	printk(KERN_ERR "===> %s\r\n", __func__);
	return 0;
}

static void spineet_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	struct spinet *priv = netdev_priv(dev);
	
	printk(KERN_ERR "===> %s\r\n", __func__);
	
	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	strlcpy(info->bus_info,
		dev_name(dev->dev.parent), sizeof(info->bus_info));
	
	/* init ipc */
	ipc_init(priv);
}

static u32 spinet_get_msglevel(struct net_device *dev)
{
	struct spinet *priv = netdev_priv(dev);
	return priv->msg_enable;
}

static void spinet_set_msglevel(struct net_device *dev, u32 val)
{
	struct spinet *priv = netdev_priv(dev);
	priv->msg_enable = val;
}

static const struct ethtool_ops spinet_ethtool_ops = {
	.get_settings	= spinet_get_settings,
	.set_settings	= spinet_set_settings,
	.get_drvinfo	= spineet_get_drvinfo,
	.get_msglevel	= spinet_get_msglevel,
	.set_msglevel	= spinet_set_msglevel,
};

static int spinet_open(struct net_device *dev)
{
	struct spinet *priv = netdev_priv(dev);

	printk(KERN_ERR "===> %s\r\n", __func__);

	if (netif_msg_drv(priv))
		printk(KERN_ERR DRV_NAME ": %s() enter\n", __func__);

	if (!is_valid_ether_addr(dev->dev_addr)) {
		if (netif_msg_ifup(priv))
			dev_err(&dev->dev, "invalid MAC address %pM\n",
				dev->dev_addr);
		return -EADDRNOTAVAIL;
	}

	return 0;
}

static int spinet_close(struct net_device *dev)
{
	printk(KERN_ERR "===> %s\r\n", __func__);
	
	return 0;
}

static netdev_tx_t spinet_send_packet(struct sk_buff *skb, struct net_device *dev)
{
	struct spinet *priv = netdev_priv(dev);

	//u8 echo_msg[] = {'E', 'C', 'H', 'O'};	
	//ipc_send(priv, echo_msg, 4);		

	if (netif_msg_tx_queued(priv))
		printk(KERN_ERR DRV_NAME ": %s() enter\n", __func__);

	/* If some error occurs while trying to transmit this
	 * packet, you should return '1' from this function.
	 * In such a case you _may not_ do anything to the
	 * SKB, it is still owned by the network queueing
	 * layer when an error is returned.  This means you
	 * may not modify any SKB fields, you may not free
	 * the SKB, etc.
	 */
	//netif_stop_queue(dev);

	/* Remember the skb for deferred processing */
	priv->tx_skb = skb;
	schedule_work(&priv->tx_work);

	return NETDEV_TX_OK;	
}						

static void spinet_multicast_list(struct net_device *dev)
{
	//printk(KERN_ERR "===> %s\r\n", __func__);
}

static int spinet_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *address = addr;
	
	printk(KERN_ERR "===> %s\r\n", __func__);
	
	if (netif_running(dev))
		return -EBUSY;
	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, address->sa_data, dev->addr_len);

	return 0;
}

static void spinet_tx_timeout(struct net_device *ndev)
{
	
}

static const struct net_device_ops spinet_netdev_ops = {
	.ndo_open		     = spinet_open,
	.ndo_stop		     = spinet_close,
	.ndo_start_xmit		 = spinet_send_packet,
	.ndo_set_rx_mode	 = spinet_multicast_list,
	.ndo_set_mac_address = spinet_set_mac_address,
	.ndo_tx_timeout		 = spinet_tx_timeout,
	.ndo_change_mtu		 = eth_change_mtu,
	.ndo_validate_addr	 = eth_validate_addr,
};

static void spinet_tx_work_handler(struct work_struct *work)
{
	struct spinet *priv = container_of(work, struct spinet, tx_work);
	
	//if (netif_msg_tx_queued(priv))
		printk(KERN_ERR DRV_NAME ": Tx Packet Len:%d\n", priv->tx_skb->len);

	//if (netif_msg_pktdata(priv))
		dump_packet(__func__, priv->tx_skb->len, priv->tx_skb->data);	
}

static void spinet_setrx_work_handler(struct work_struct *work)
{
	
}

static void spinet_restart_work_handler(struct work_struct *work)
{
	
}

static int spinet_gpio_init(struct spinet *priv)
{
	struct device *dev = (struct device *)&priv->spi->dev;
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

static void spinet_gpio_deinit(struct spinet *priv)
{
	gpio_free(priv->status_out);
	gpio_free(priv->send_request);
	gpio_free(priv->status_in);	
}

static void spinet_free_buffer(struct spinet *priv)
{	
	if(priv->tx_buff) {
		kfree(priv->tx_buff);
		priv->tx_buff = NULL;
	}

	if(priv->rx_buff) {
		kfree(priv->rx_buff);
		priv->rx_buff = NULL;
	}
}

static int spinet_probe(struct spi_device *spi)
{
	const struct of_device_id *match;
	struct net_device *dev;
	struct spinet *priv;
	int ret = 0;	
	
	dev = alloc_etherdev(sizeof(struct spinet));
	if (!dev) {
		ret = -ENOMEM;
		goto error_alloc;
	}
	priv = netdev_priv(dev);
	
	priv->netdev = dev;	/* priv to netdev reference */
	priv->spi = spi;	/* priv to spi reference */	
	priv->msg_enable = netif_msg_init(debug.msg_enable, SPINET_MSG_DEFAULT);
						
	/* allocate spi tx/rx buffers */
	priv->tx_buff = devm_kzalloc(&spi->dev, IPC_TRANSFER_LEN, GFP_KERNEL);

	if (!priv->tx_buff) {
		ret = -ENOMEM;
		dev_err(&dev->dev, DRV_NAME " memory allocation failed\n");
		goto free_net_dev;
	}

	priv->rx_buff = devm_kzalloc(&spi->dev, IPC_TRANSFER_LEN, GFP_KERNEL);

	if (!priv->rx_buff) {
		ret = -ENOMEM;
		dev_err(&dev->dev, DRV_NAME " memory allocation failed\n");
		goto free_spi_buffers;
	}

	/* allocate tx fifo buffer */
	ret = kfifo_alloc(&priv->ipc_tx_fifo, FIFO_BUF_SIZE, GFP_KERNEL); 
	if(ret){
		ret = -ENOMEM;
		dev_err(&dev->dev, DRV_NAME " fifo buffer allocation failed\n");
		goto free_spi_buffers;
	}	

	mutex_init(&priv->lock);
	INIT_WORK(&priv->spi_work, spinet_spi_work_handler);
	INIT_WORK(&priv->tx_work, spinet_tx_work_handler);
	INIT_WORK(&priv->setrx_work, spinet_setrx_work_handler);
	INIT_WORK(&priv->irq_work, spinet_irq_work_handler);
	INIT_WORK(&priv->restart_work, spinet_restart_work_handler);
	
	/* spi mode setting */
	spi->mode = SPI_MODE;
	spi->bits_per_word = 8;

	/* setup spi */
	ret = spi_setup(spi);
	if (ret) {
		dev_err(&spi->dev, "setup spi failed\r\n");
		goto free_fifo_buffer;
	}	
	
	/* init spi transfer */
	priv->spi_transfer.tx_buf = priv->tx_buff; 
	priv->spi_transfer.rx_buf = priv->rx_buff;
	priv->spi_transfer.len = IPC_TRANSFER_LEN;
	priv->spi_transfer.cs_change = 0;
	
	spi_set_drvdata(spi, priv);	/* spi to priv reference */	
	SET_NETDEV_DEV(dev, &spi->dev);
	
	/* init spi message */
	spi_message_init(&priv->spi_msg);	
	
	/* get gpio pin number from device tree */
	match = of_match_device(of_match_ptr(spinet_of_match), &spi->dev);

	if(match) {
		priv->status_in = of_get_named_gpio(spi->dev.of_node, "status_in", 0);
		priv->status_out = of_get_named_gpio(spi->dev.of_node, "status_out", 0);
		priv->send_request = of_get_named_gpio(spi->dev.of_node, "send_request", 0);
		priv->irq_in = of_get_named_gpio(spi->dev.of_node, "irq_in", 0);		
	}		
	
	/* setup gpio */
	ret = spinet_gpio_init(priv);
	if (ret) {
		goto free_fifo_buffer;
	}

	/* register gpio interrupt */
	ret = request_threaded_irq(gpio_to_irq(priv->irq_in), 
							NULL, 
							spinet_gpio_irq,
							IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
							spi->dev.driver->name, priv);
	if (ret) {
		dev_err(&spi->dev, "could not regster irq %d\r\n", gpio_to_irq(priv->irq_in));
		goto gpio_deinit;
	}	

	/* store gpio irq */
	priv->gpio_irq = gpio_to_irq(priv->irq_in);
	
	/* we will enable it later */
	disable_irq_nosync(priv->gpio_irq);

	/* init resouce lock */
	spin_lock_init(&priv->buff_lock);

	/* init netdev methods */
	dev->if_port = IF_PORT_10BASET;
	dev->netdev_ops = &spinet_netdev_ops;
	dev->watchdog_timeo = TX_TIMEOUT;
	dev->ethtool_ops = &spinet_ethtool_ops;

	ret = register_netdev(dev);
	if (ret) {
		if (netif_msg_probe(priv))
			dev_err(&spi->dev, "register netdev " DRV_NAME
				" failed (ret = %d)\n", ret);
		goto free_gpio_irq;
	}
	dev_err(&dev->dev, DRV_NAME " driver registered\n");	
	
	return 0;
	
free_gpio_irq:
	free_irq(priv->gpio_irq, priv);

gpio_deinit:
	spinet_gpio_deinit(priv);

free_fifo_buffer:
	kfifo_free(&priv->ipc_tx_fifo);

free_spi_buffers:
	spinet_free_buffer(priv);

free_net_dev:
	free_netdev(dev);

error_alloc:
	return ret;
}

static int spinet_remove(struct spi_device *spi)
{
	struct spinet *priv = spi_get_drvdata(spi);
	
	if (netif_msg_drv(priv))
		printk(KERN_ERR DRV_NAME ": remove\n");

	unregister_netdev(priv->netdev);
	spinet_free_buffer(priv);
	kfifo_free(&priv->ipc_tx_fifo);
	free_irq(priv->gpio_irq, priv);
	spinet_gpio_deinit(priv);
	free_netdev(priv->netdev);	
	priv = NULL;

	return 0;
}

static struct spi_driver spinet_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = spinet_of_match,
	},
	.id_table = spinet_id_table,
	.probe = spinet_probe,
	.remove = spinet_remove,
};

static int __init spinet_init(void)
{
	msec20_to_jiffies = msecs_to_jiffies(20);

	return spi_register_driver(&spinet_driver);
}

static void __exit spinet_exit(void)
{
	spi_unregister_driver(&spinet_driver);
}

module_init(spinet_init);
module_exit(spinet_exit);

MODULE_DESCRIPTION(DRV_NAME " spi eth emulated driver");
MODULE_AUTHOR("Henry Dang");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:" DRV_NAME);

