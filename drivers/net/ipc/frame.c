/*
 *  Copyright (C) : 2018
 *  File name     : frame.c
 *  Author        : Dang Minh Phuong
 *  Email         : kamejoko80@yahoo.com
 *
 *  This program is free software, you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
 
#include <linux/kernel.h>
#include <linux/module.h>
#include "frame.h"

u8 ipc_frame_crc8(u8 *data, u16 len)
{
	unsigned crc = 0;
	int i, j;

	/* using x^8 + x^2 + x + 1 polynomial */
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

bool ipc_frame_create(ipc_frame_t *frame, u8 *data, u16 len)
{
	bool ret = false;

	if(len <= IPC_DATA_MAX_LEN) {
		/* reset IPC frame */
		memset((void *)frame, 0, sizeof(ipc_frame_t));

		/* create IPC frame header */
		frame->header.soh  = IPC_FRAME_SOH;
		frame->header.len  = len;
		frame->header.crc8 = ipc_frame_crc8((u8 *)&frame->header, 3);

		/* create IPC frame data and CRC8 */
		memcpy(frame->data, data, len);
		frame->crc8 = ipc_frame_crc8(frame->data, len);

		ret = true;
	} else {
        printk(KERN_ERR "Error data length exceeds allow max length\r\n");
    }

	return ret;
}

bool ipc_frame_check(ipc_frame_t *frame)
{
    bool ret = false;

	if(frame->header.soh != IPC_FRAME_SOH) {
		// printk("Error IPC frame header SOH\r\n");
	} else if(ipc_frame_crc8((u8 *)&frame->header, 3) != frame->header.crc8) {
		// printk("Error IPC frame header\r\n");
	} else if(ipc_frame_crc8(frame->data, frame->header.len) != frame->crc8) {
		// printk("Error IPC frame data\r\n");
	} else {
		ret = true;
	}
	return ret;
}

void ipc_frame_print(ipc_frame_t *frame)
{
	printk(KERN_EMERG "===== IPC Frame Info =====\r\n");
	printk(KERN_EMERG "Header:\r\n");
	printk(KERN_EMERG "  Soh : 0x%X\r\n", frame->header.soh);
	printk(KERN_EMERG "  Len : %d\r\n", frame->header.len);
	printk(KERN_EMERG "  Data: ");
	//printk(KERN_EMERG "%s", frame->data);

#if 1
	int i;

	for (i = 0; i < frame->header.len; i++) {
		printk(KERN_EMERG "0x%X ", frame->data[i]);
	}
#endif
	printk("\r\n");
}

EXPORT_SYMBOL(ipc_frame_print);
EXPORT_SYMBOL(ipc_frame_create);
EXPORT_SYMBOL(ipc_frame_check);

MODULE_AUTHOR("Dang Minh Phuong <kamejoko80@yahoo.com>");
MODULE_DESCRIPTION("IPC driver");
MODULE_LICENSE("GPL v2");