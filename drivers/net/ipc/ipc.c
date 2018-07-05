/*
 *  Copyright (C) : 2018
 *  File name     : ipc.c
 *  Author        : Dang Minh Phuong
 *  Email         : kamejoko80@yahoo.com
 *
 *  This program is free software, you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
 
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/freezer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/kfifo.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <net/sock.h>
#include <linux/skbuff.h>
#include <linux/netlink.h>

#include "frame.h"

#define DEVICE_NAME "ipc"
#define NETLINK_USER 31
#define NETLINK_GROUP 1
#define MAX_NL_PAYLOAD 1024

struct ipc_priv {
	struct spi_device *spi;
	struct mutex ipc_lock;
	struct workqueue_struct *wq;
	struct work_struct ipc_work;
	struct kfifo ipc_rx_fifo;
	struct kfifo ipc_tx_fifo;
	struct spi_transfer spi_transfer;
	struct spi_message spi_msg;
	struct netlink_kernel_cfg nl_cfg;
	struct sock *nl_sk;
	u8 *spi_tx_buf;
	u8 *spi_rx_buf;
	int rqst_in;
	int rqst_out;
};

static struct ipc_priv *g_priv = NULL; 

static const struct of_device_id ipc_of_match[] = {
	{
		.compatible	= "pdtech,ipc",
	},
	{ }
};

MODULE_DEVICE_TABLE(of, ipc_of_match);

static const struct spi_device_id ipc_id_table[] = {
	{
		.name = "ipc",
	},
	{ }
};

MODULE_DEVICE_TABLE(spi, ipc_id_table);

static int ipc_gpio_init(struct ipc_priv *priv)
{
	struct device *dev = (struct device *)&priv->spi->dev;
	int err = 0;
	
	err = gpio_request(priv->rqst_out, "rqst_out");
	if (err) {
		dev_err(dev, "gpio is busy\r\n");
		return err;
	}

	err = gpio_direction_output(priv->rqst_out, 1);
	if (err) {
		dev_err(dev, "gpio set ouput failed\r\n");
		return err;
	}
	return err;
}

static void ipc_rqst_out(struct ipc_priv *priv)
{
	gpio_set_value(priv->rqst_out, 0);
	gpio_set_value(priv->rqst_out, 1);
}

void ipc_send(struct ipc_priv *priv, u8 *data, u16 len)
{
	ipc_frame_t *frame = (ipc_frame_t *)priv->spi_tx_buf;

	/* check if there is enough space in the tx fifo */
	if(kfifo_avail(&priv->ipc_tx_fifo) >= IPC_TRANSFER_LEN){
		if (ipc_frame_create(frame, data, len)) {
			/* put data into the tx fifo */
			mutex_lock_interruptible(&priv->ipc_lock);
			kfifo_in(&priv->ipc_tx_fifo, frame, IPC_TRANSFER_LEN);
			mutex_unlock(&priv->ipc_lock);
			/* notify spi slaver to start the transfer */
			ipc_rqst_out(priv);
		}
	}
}

void ipc_transfer_complete(void *context)
{
	struct ipc_priv *priv = (struct ipc_priv *)context;
	ipc_frame_t *frame = (ipc_frame_t *)priv->spi_rx_buf;

	/* check valid incomming frame */
	if(ipc_frame_check(frame)) {
		/* check if there is enough space in the rx fifo */
		if(kfifo_avail(&priv->ipc_rx_fifo) >= IPC_TRANSFER_LEN){
			/* put data into rx Fifo */
			mutex_lock_interruptible(&priv->ipc_lock);
			kfifo_in(&priv->ipc_rx_fifo, frame, IPC_TRANSFER_LEN);
			mutex_unlock(&priv->ipc_lock);
			/* notify there is incomming data */
			queue_work(priv->wq, &priv->ipc_work);
		}
	}
}

void ipc_spi_transfer_queue(struct ipc_priv *priv)
{
	struct device *dev = (struct device *)&priv->spi->dev;
	int err;

	/* clear spi tx buffer */
	memset((void *)priv->spi_tx_buf, 0, IPC_TRANSFER_LEN);

	/* check if there are data to transfer */
	if(kfifo_len(&priv->ipc_tx_fifo) >= IPC_TRANSFER_LEN)
	{
		mutex_lock_interruptible(&priv->ipc_lock);
		kfifo_out(&priv->ipc_tx_fifo, priv->spi_tx_buf, IPC_TRANSFER_LEN);
		mutex_unlock(&priv->ipc_lock);
	}

	/* init spi message */
	spi_message_init(&priv->spi_msg);

	/* setup spi transfer complete */
	priv->spi_msg.complete = &ipc_transfer_complete;
	priv->spi_msg.context = (void *)priv;

	/* add spi transfer to the list */
	spi_message_add_tail(&priv->spi_transfer, &priv->spi_msg);	

	/* queue spi transfer */
	err = spi_async(priv->spi, &priv->spi_msg);
	if (err) {
		dev_err(dev, "spi queue failed: err = %d\n", err);
	}
}

static void ipc_nl_recv_msg(struct sk_buff *skb)
{
	struct nlmsghdr *nlh = (struct nlmsghdr *)skb->data;

	/* send data via IPC interface */
	ipc_send(g_priv, (u8 *)nlmsg_data(nlh), nlmsg_len(nlh));
}

static void ipc_nl_send_msg(struct ipc_priv *priv, ipc_frame_t *frame)
{
	struct device *dev = (struct device *)&priv->spi->dev;
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	int err;

	/* allocate netlink message */
	skb = nlmsg_new(IPC_TRANSFER_LEN, GFP_KERNEL);
	if (!skb) {
		dev_err(dev, "failed to allocate new skb\n");
		return;
	}
	
	nlh = nlmsg_put(skb, 0, 0, NLMSG_DONE, frame->header.len, 0);
	NETLINK_CB(skb).portid = 0; /* from kernel */
	NETLINK_CB(skb).dst_group = NETLINK_GROUP; /* multicast group */
	strncpy(nlmsg_data(nlh), (void *)frame->data, frame->header.len);

	err = netlink_broadcast(g_priv->nl_sk, skb, 0, NETLINK_GROUP, GFP_KERNEL);
	if (err < 0)
		dev_err(dev, "error while ipc sends msg to user\n");
}

static void ipc_work_handler(struct work_struct *ws)
{
	struct ipc_priv *priv = container_of(ws, struct ipc_priv, ipc_work);
	ipc_frame_t *frame = (ipc_frame_t *)priv->spi_rx_buf;

	/* read out rx data */
	if(kfifo_len(&priv->ipc_rx_fifo) >= IPC_TRANSFER_LEN) {
		mutex_lock_interruptible(&priv->ipc_lock);
		kfifo_out(&priv->ipc_rx_fifo, frame, IPC_TRANSFER_LEN);
		mutex_unlock(&priv->ipc_lock);
		//ipc_frame_print(frame);
		ipc_nl_send_msg(priv, frame);
	}
	
	/* 
	 * check if there is remained tx data then
	 * trigger the ipc transaction again
	 */
	if(kfifo_avail(&priv->ipc_tx_fifo) >= IPC_TRANSFER_LEN){
		/* notify spi slaver to start the transfer */
		ipc_rqst_out(priv);
	}

}

static irqreturn_t ipc_rqst_in_isr(int irq, void *devid)
{
	struct ipc_priv *priv = (struct ipc_priv *)devid;

	/* queue spi transfer */
	ipc_spi_transfer_queue(priv);

	return IRQ_HANDLED;
}

static int ipc_probe(struct spi_device *spi)
{
	const struct of_device_id *match;
	struct ipc_priv *priv;
	int err = 0;

	priv = kzalloc(sizeof(struct ipc_priv), GFP_KERNEL);

	if (!priv) {
		err = -ENOMEM;
		dev_err(&spi->dev, "could not allocate memory\r\n");
		goto exit;
	}

	/* allocate spi data buffer */
	priv->spi_tx_buf = devm_kzalloc(&spi->dev, IPC_TRANSFER_LEN,
					GFP_KERNEL);
	if (!priv->spi_tx_buf) {
		err = -ENOMEM;
		goto exit_free;
	}

	/* initialize mutex lock */
	mutex_init(&priv->ipc_lock);

	priv->spi_rx_buf = devm_kzalloc(&spi->dev, IPC_TRANSFER_LEN,
					GFP_KERNEL);
	if (!priv->spi_rx_buf) {
		err = -ENOMEM;
		goto exit_free_spi_tx_buf;
	}

	/* allocate tx fifo buffer */
	err = kfifo_alloc(&priv->ipc_tx_fifo, FIFO_BUF_SIZE, GFP_KERNEL); 
	if(err){
		err = -ENOMEM;
		goto exit_free_spi_rx_buf;
	}

	/* allocate rx fifo buffer */
	err = kfifo_alloc(&priv->ipc_rx_fifo, FIFO_BUF_SIZE, GFP_KERNEL); 
	if(err){
		err = -ENOMEM;
		goto exit_free_ipc_tx_fifo;
	}

	match = of_match_device(of_match_ptr(ipc_of_match), &spi->dev);

	/* get gpio pin number from device tree */
	if(match) {
		priv->rqst_in = of_get_named_gpio(spi->dev.of_node, "rqst_in", 0);
		priv->rqst_out = of_get_named_gpio(spi->dev.of_node, "rqst_out", 0);
	}

	/* store spi infomation */
	priv->spi = spi;

	/* spi mode setting */
	spi->mode = SPI_MODE_0;
	
	/* store driver data */
	spi_set_drvdata(spi, priv);
	g_priv = priv;

	/* setup spi */
	err = spi_setup(spi);
	if (err) {
		dev_err(&spi->dev, "setup spi failed\r\n");
		goto exit_free_ipc_rx_fifo;
	}

	/* init spi transfer */
	priv->spi_transfer.tx_buf = priv->spi_tx_buf; 
	priv->spi_transfer.rx_buf = priv->spi_rx_buf;
	priv->spi_transfer.len = IPC_TRANSFER_LEN;
	priv->spi_transfer.cs_change = 0;

	/* init ipc workqueue */
	priv->wq = create_freezable_workqueue("ipc_wq");
	INIT_WORK(&priv->ipc_work, ipc_work_handler);

	/* setup gpio */
	err = ipc_gpio_init(priv);
	if (err) {
		goto exit_free_ipc_rx_fifo;
	}

	/* register gpio interrupt */
	err = request_threaded_irq(gpio_to_irq(priv->rqst_in), NULL, ipc_rqst_in_isr,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   spi->dev.driver->name, priv);
	if (err) {
		dev_err(&spi->dev, "could not regster irq %d\r\n", gpio_to_irq(priv->rqst_in));
		goto exit_free_ipc_rx_fifo;
	}

	/* setting netlink socket */
	priv->nl_cfg.input = &ipc_nl_recv_msg;
	priv->nl_cfg.groups = NETLINK_GROUP;
	priv->nl_sk = netlink_kernel_create(&init_net, NETLINK_USER, &priv->nl_cfg);
	if(!priv->nl_sk) {
		dev_err(&spi->dev, "could not allocate nl socket\r\n");
		goto exit_free_ipc_rx_fifo;
	}

	return 0;

exit_free_ipc_rx_fifo:	
	kfifo_free(&priv->ipc_rx_fifo);

exit_free_ipc_tx_fifo:	
	kfifo_free(&priv->ipc_tx_fifo);

exit_free_spi_rx_buf:
	kfree(priv->spi_rx_buf);

exit_free_spi_tx_buf:
	kfree(priv->spi_tx_buf);

exit_free:
	kfree(priv);

exit:
	return err;
}

static int ipc_remove(struct spi_device *spi)
{
	struct ipc_priv *priv = spi_get_drvdata(spi);

	/* free the workqueue */
	destroy_workqueue(priv->wq);

	netlink_kernel_release(priv->nl_sk);
	kfifo_free(&priv->ipc_rx_fifo);
	kfifo_free(&priv->ipc_tx_fifo);
	kfree(priv->spi_rx_buf);
	kfree(priv->spi_tx_buf);

	if (priv)
		kfree(priv);

	return 0;
}

static int __maybe_unused ipc_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused ipc_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(ipc_pm_ops, ipc_suspend, ipc_resume);

static struct spi_driver ipc_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = ipc_of_match,
		.pm = &ipc_pm_ops,
	},
	.id_table = ipc_id_table,
	.probe = ipc_probe,
	.remove = ipc_remove,
};

module_spi_driver(ipc_driver);

MODULE_AUTHOR("Dang Minh Phuong <kamejoko80@yahoo.com>");
MODULE_DESCRIPTION("IPC driver");
MODULE_LICENSE("GPL v2");