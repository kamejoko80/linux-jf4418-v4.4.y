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

#include "spinet.h"
#include "ncovif.h"
#include "ipc.h"

/* function prototypes */
void spi_write_async(struct spinet *priv, u8 *buf);

/* use ethtool to change the level for any given device */
static struct {
	u32 msg_enable;
} debug = { -1 };

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

/* NCOVIF debugging implementation */

uint8_t  my_macaddr[6] = {0x51, 0x80, 0x21, 0xfe, 0xad, 0xd2};
uint8_t  my_ipaddr[4]  = {0x0A, 0x00, 0x00, 0x02};

uint8_t  ex_macaddr[6] = {0xA6, 0x99, 0x91, 0xAD, 0x9D, 0x6F};
uint8_t  ex_ipaddr[4]  = {0x0A, 0x00, 0x00, 0x01};

struct ethIIhdr eth_frame_dst;

volatile bool arp_request = false;
volatile bool icmp_request = false;

void eth_header_print(struct ethIIhdr *frame);
void arp_header_print(struct arphdr *arp);
int icmp_build_package(struct iphdr *sender_iph, uint8_t *icmpd, uint16_t len);
uint8_t* ip_output_standalone(struct ethIIhdr *eth, uint8_t protocol, uint16_t ip_id,
								uint32_t saddr, uint32_t daddr, uint16_t payloadlen);

void ipc_network_input(struct spinet *priv, uint8_t *data, uint16_t len)
{
	struct net_device *ndev = priv->netdev;
	struct sk_buff *skb = NULL;
	unsigned long flags;

	skb = netdev_alloc_skb(ndev, len + NET_IP_ALIGN);
	if (!skb) {
			dev_err(&ndev->dev, "out of memory for Rx'd frame\n");
			ndev->stats.rx_dropped++;
	} else {
		skb_reserve(skb, NET_IP_ALIGN);
		spin_lock_irqsave(&priv->buff_lock, flags);
		memcpy(skb_put(skb, len), data, len);
		spin_unlock_irqrestore(&priv->buff_lock, flags);
		skb->protocol = eth_type_trans(skb, ndev);
		/* update statistics */
		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += len;
		netif_rx_ni(skb);
	}
}

void send_icmp_echo(struct spinet *priv)
{
	if(icmp_request){
		ipc_network_input(priv, (uint8_t *)&eth_frame_dst, 98);
		icmp_request = false;
	}
}

void send_arp_response(struct spinet *priv)
{
	if(arp_request)
	{
		printk(KERN_ERR "send_arp_response\r\n");
		ipc_network_input(priv, (uint8_t *)&eth_frame_dst, 42);
		arp_request = false;
	}
}

void build_arp_reply_package(struct ethIIhdr *eth_frame_dst, struct ethIIhdr *eth_frame_src)
{
	struct arphdr *arp_dest = (struct arphdr *)eth_frame_dst->data;
	struct arphdr *arp_src  = (struct arphdr *)eth_frame_src->data;

	/* build response message */
	memcpy(eth_frame_dst->h_source, my_macaddr, 6);
	memcpy(eth_frame_dst->h_dest, eth_frame_src->h_source, 6);
	eth_frame_dst->h_proto = eth_frame_src->h_proto;

	arp_dest->ar_hrd = arp_src->ar_hrd;
	arp_dest->ar_pro = arp_src->ar_pro;
	arp_dest->ar_hln = arp_src->ar_hln;
	arp_dest->ar_pln = arp_src->ar_pln;
	arp_dest->ar_op = __swap16(ARPOP_REPLY);
	memcpy(arp_dest->ar_sha, my_macaddr, 6);
	memcpy(arp_dest->ar_sip, my_ipaddr, 4);
	memcpy(arp_dest->ar_tha, arp_src->ar_sha, 6);
	memcpy(arp_dest->ar_tip, arp_src->ar_sip , 4);
}

void arp_frame_process(struct spinet *priv, struct ethIIhdr *eth_frame)
{
	struct arphdr *arp_frame = (struct arphdr *)eth_frame->data;

	switch (arp_frame->ar_op){
		case __swap16(ARPOP_REQUEST):
			build_arp_reply_package(&eth_frame_dst, eth_frame);
			arp_request = true;
			schedule_work(&priv->spi_work);
		break;

		case __swap16(ARPOP_REPLY):
		break;

		default:
		break;
	}
}

void icmp_frame_process(struct spinet *priv, struct ethIIhdr *eth_frame)
{
	struct iphdr *iph = (struct iphdr *)eth_frame->data;
	struct icmphdr *icmph = (struct icmphdr *) IP_NEXT_PTR(iph);

	uint16_t len, data_len;

	switch (icmph->icmp_type){
		case ICMP_ECHO:
			len = ntohs(iph->tot_len);
			data_len = (uint16_t)(len-(iph->ihl<<2)-sizeof(struct icmphdr));
			icmp_build_package(iph, (uint8_t *)(icmph + 1), data_len);
			icmp_request = true;
			schedule_work(&priv->spi_work);
		break;

		default:

		break;
	}
}

void ip_frame_process(struct spinet *priv, struct ethIIhdr *eth_frame)
{
	struct iphdr *ip_frame = (struct iphdr *)eth_frame->data;

	switch(ip_frame->protocol){
		case IPPROTO_ICMP:
			icmp_frame_process(priv, eth_frame);
		break;
		default:
		break;
	}
}

void eth_frame_process(struct spinet *priv, struct ethIIhdr *eth_frame)
{
	switch (eth_frame->h_proto){
		case __swap16(ETH_P_ARP):
			arp_frame_process(priv, eth_frame);
		break;

		case __swap16(ETH_P_IP):
			ip_frame_process(priv, eth_frame);
		break;

		default:

		break;
	}
}

uint16_t icmp_checksum(uint16_t *icmph, int len)
{
	uint16_t ret = 0;
	uint32_t sum = 0;
	uint16_t odd_byte;

	while (len > 1) {
		sum += *icmph++;
		len -= 2;
	}

	if (len == 1) {
		*(uint8_t*)(&odd_byte) = *(uint8_t*)icmph;
		sum += odd_byte;
	}

	sum =  (sum >> 16) + (sum & 0xffff);
	sum += (sum >> 16);
	ret =  ~sum;

	return ret;
}

uint8_t* ip_output_standalone(struct ethIIhdr *eth, uint8_t protocol, uint16_t ip_id,
		                      uint32_t saddr, uint32_t daddr, uint16_t payloadlen)
{
	struct iphdr *iph = (struct iphdr *)eth->data;

	/* build up eth header */
	memcpy(eth->h_source, my_macaddr, 6);
	memcpy(eth->h_dest, ex_macaddr, 6);
	eth->h_proto = __swap16(ETH_P_IP);

	/* build up ip header */
	iph->ihl = IP_HEADER_LEN >> 2;
	iph->version = 4;
	iph->tos = 0;
	iph->tot_len = htons(IP_HEADER_LEN + payloadlen);
	iph->id = ip_id;
	iph->frag_off = htons(IP_DF);
	iph->ttl = 64;
	iph->protocol = protocol;
	iph->saddr = saddr;
	iph->daddr = daddr;
	iph->check = 0;

	/* calculate checksum */
	iph->check = ip_fast_csum(iph, iph->ihl);

	return (uint8_t *)(iph + 1);
}

int icmp_build_package(struct iphdr *sender_iph, uint8_t *icmpd, uint16_t len)
{
	struct icmphdr *icmph;
	struct icmphdr *sender_icmph = (struct icmphdr *) IP_NEXT_PTR(sender_iph);

	icmph = (struct icmphdr *)ip_output_standalone(&eth_frame_dst,
							IPPROTO_ICMP,
							sender_iph->id,
							sender_iph->daddr,
							sender_iph->saddr,
							sizeof(struct icmphdr) + len);
	if (!icmph)
		return -1;

	/* Fill in the icmp echo reply header */
	icmph->icmp_type = ICMP_ECHOREPLY;
	icmph->icmp_code = sender_icmph->icmp_code;
	icmph->icmp_checksum = 0;

	icmph->un.echo.icmp_id = sender_icmph->un.echo.icmp_id;
	icmph->un.echo.icmp_sequence = sender_icmph->un.echo.icmp_sequence;

	/* Fill in the icmp data */
	if (len > 0){
		memcpy((void *)(icmph + 1), icmpd, len);
	}

	/* Calculate ICMP Checksum with header and data */
	icmph->icmp_checksum = icmp_checksum((uint16_t *)icmph, sizeof(struct icmphdr) + len);

	return 0;
}

/* IPC implementation */

/* IPC transfer complete status */
static bool ipc_transfer_complete = true;
static bool is_sending = false;

void ipc_frame_print(ipc_frame_t *frame)
{
	int i;

	printk(KERN_ERR "===== IPC Frame Info =====\r\n");
	printk(KERN_ERR "Header:\r\n");
	printk(KERN_ERR "  Soh : 0x%X\r\n", frame->header.soh);
	printk(KERN_ERR "  Len : %d\r\n", frame->header.len);
	printk(KERN_ERR "  Data:\r\n");

	for (i = 0; i < frame->header.len; i++) {

		if(frame->data[i] <= 0x0F){
			printk(KERN_ERR "0%X ", frame->data[i]);
		}else{
			printk(KERN_ERR "%X ", frame->data[i]);
		}
	}

	printk(KERN_ERR "\r\n");
}

void ipc_dump_package(uint8_t *package, uint16_t len)
{
	uint16_t i;

	printk(KERN_ERR "===== Package dumping =====\r\n");

	for (i = 0; i < len; i++){
		if(package[i] <= 0x0F){
			printk(KERN_ERR "0%X ", package[i]);
		}else{
			printk(KERN_ERR "%X ", package[i]);
		}
	}

	printk(KERN_ERR "\r\n");
}

void delay_us(u32 us)
{
	u32 i, j;

	for(i = 0; i < (1000000/CPU_MHZ); i++){
		for(j = 0; j < us; j++);
	}
}

u8 ipc_frame_crc8(u8 *data, size_t len)
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

bool ipc_frame_create(struct spinet *priv, ipc_frame_t *frame, u8 *data, size_t len)
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

bool ipc_frame_check(ipc_frame_t *frame)
{
	bool ret = false;

	if(frame->header.soh != IPC_FRAME_SOH) {
		return ret;
	} else if(ipc_frame_crc8((u8 *)&frame->header, 3) != frame->header.crc8) {
		return ret;
	} else if(ipc_frame_crc8(frame->data, frame->header.len) != frame->crc8) {
		return ret;
	} else {
		ret = true;
	}

	return ret;
}

void ipc_init(struct spinet *priv)
{
	set_master_ready();
	ipc_transfer_complete = true;
	enable_irq(priv->gpio_irq);
}

ipc_status_t ipc_send(struct spinet *priv, u8 *data, size_t len)
{

#ifdef NCOVIF_ENABLE
	ipc_status_t ret = IPC_OK;

	/* internal received */
	eth_frame_process(priv, (struct ethIIhdr *)data);	
#else

	ipc_frame_t *frame = (ipc_frame_t *)priv->tx_buff;
	ipc_status_t ret = IPC_OK;

	if(!ipc_transfer_complete) {
		return IPC_BUSY;
	}

	set_master_busy();

	if(!is_slaver_busy()) {
		if(ipc_frame_create(priv, frame, data, len)) {
			is_sending = true;
			master_send_request();
			mdelay(WAITTIME);
			ipc_transfer_complete = false;
			spi_write_async(priv, (u8 *)frame);
			ret = IPC_OK;
		} else {
			printk(KERN_ERR "===== ipc frame create error =====\r\n");
			set_master_ready();
			ret = IPC_ERR;
		}
	} else {
		//printk(KERN_ERR "===== is_slaver_busy =====\r\n");
		set_master_ready();
		ret = IPC_BUSY;
	}
#endif

    return ret;
}

void ipc_cmd_sync(struct spinet *priv)
{
	ipc_init(priv);
	master_send_request();
	mdelay(WAITTIME);
}

void ipc_receive_callback(struct spinet *priv)
{
	ipc_frame_t *frame = (ipc_frame_t *)priv->rx_buff;
	struct net_device *ndev = priv->netdev;
	struct sk_buff *skb = NULL;
	unsigned long flags;
	int len;
	
	if(ipc_frame_check(frame)){
		//ipc_frame_print(frame);
		len = frame->header.len;
		skb = netdev_alloc_skb(ndev, len + NET_IP_ALIGN);
		if (!skb) {
			dev_err(&ndev->dev, "skb allocation failed\n");
			ndev->stats.rx_dropped++;
		} else {
			skb_reserve(skb, NET_IP_ALIGN);
			spin_lock_irqsave(&priv->buff_lock, flags);
			memcpy(skb_put(skb, len), frame->data, len);
			spin_unlock_irqrestore(&priv->buff_lock, flags);
			skb->protocol = eth_type_trans(skb, ndev);
			/* update statistics */
			ndev->stats.rx_packets++;
			ndev->stats.rx_bytes += len;
			netif_rx_ni(skb);
		}
	}
}

void spinet_spi_work_handler(struct work_struct *work)
{
	struct spinet *priv = container_of(work, struct spinet, spi_work);

#ifdef NCOVIF_ENABLE
	send_arp_response(priv);
	send_icmp_echo(priv);	
#else
	ipc_receive_callback(priv);
	ipc_transfer_complete = true;	
	set_master_ready();
#endif
}

void spinet_irq_work_handler(struct work_struct *work)
{
	struct spinet *priv = container_of(work, struct spinet, irq_work);
	
	if(!is_slaver_busy()){
		printk(KERN_ERR "===> sync cmd\r\n");
		ipc_init(priv);
	}else if(ipc_transfer_complete){
		set_master_busy();
		ipc_transfer_complete = false;
		spi_write_async(priv, NULL);
	}	
}

irqreturn_t spinet_gpio_irq(int irq, void *devid)
{
	struct spinet *priv = (struct spinet *)devid;

	schedule_work(&priv->irq_work);

	return IRQ_HANDLED;
}

void spi_transfer_complete(void *context)
{
	struct spinet *priv = (struct spinet *)context;

	if(is_sending) {
		is_sending = false;
		ipc_transfer_complete = true;
		set_master_ready();
	} else {
		schedule_work(&priv->spi_work);
	}
}

void spi_write_async(struct spinet *priv, u8 *buf)
{
	struct device *dev = (struct device *)&priv->spi->dev;
	unsigned long flags;
	int err;

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
	//struct spinet *priv = netdev_priv(dev);
	//printk(KERN_ERR "===> %s\r\n", __func__);
	
	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	strlcpy(info->bus_info,
		dev_name(dev->dev.parent), sizeof(info->bus_info));
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

	/* init ipc */
	ipc_init(priv);

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
	ipc_status_t ipc_status;
	netdev_tx_t ret = NETDEV_TX_OK;

	if (netif_msg_tx_queued(priv))
		printk(KERN_ERR DRV_NAME ": %s() enter\n", __func__);

	if(skb != NULL) {
		priv->tx_skb = skb;
		ipc_status = ipc_send(priv, priv->tx_skb->data, priv->tx_skb->len);
		if(ipc_status == IPC_OK) {
			dev->stats.tx_packets++;
			dev->stats.tx_bytes += priv->tx_skb->len;	
			dev_kfree_skb(priv->tx_skb);
			priv->tx_skb = NULL;
		}else{
			ret = NETDEV_TX_BUSY;	
		}	
	}

	return ret;	
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
	
	printk(KERN_ERR DRV_NAME ": Tx Packet Len:%d\n", priv->tx_skb->len);
	
	ipc_send(priv, priv->tx_skb->data, priv->tx_skb->len);
	
	if (priv->tx_skb) {
		dev_kfree_skb(priv->tx_skb);
		priv->tx_skb = NULL;
	}
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
		dev_err(&dev->dev, DRV_NAME " net dev allocation failed\n");
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
		dev_err(&spi->dev, "gpio init failed\r\n");
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

