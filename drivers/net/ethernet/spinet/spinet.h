#ifndef _SPINET_H_
#define _SPINET_H_

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

/*
 * To test when NCOVIF enabled:
 *
 * sudo ifconfig eth1 down
 * sudo ifconfig eth1 hw ether a6:99:91:ad:9d:6f
 * sudo ifconfig eth1 10.0.0.1 up
 * ping -i 0.2 -I eth1 10.0.0.2
 *
 */
 
/* Enable NCOVIF debugging */
//#define NCOVIF_ENABLE 

#define DRV_NAME    "spinet"
#define DRV_VERSION "1.01"

#define SPI_MODE (SPI_MODE_0)

#define SPINET_MSG_DEFAULT\
   (NETIF_MSG_PROBE | NETIF_MSG_IFUP | NETIF_MSG_IFDOWN | NETIF_MSG_LINK)

#define TX_TIMEOUT	(4 * HZ)

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

	u8 bank;            /* current register bank selected */
	u16 next_pk_ptr;    /* next packet pointer within FIFO */
	u16 max_pk_counter; /* statistics: max packet counter */
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

#endif
