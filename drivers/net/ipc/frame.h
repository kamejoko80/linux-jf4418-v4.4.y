/*
 *  Copyright (C) : 2018
 *  File name     : frame.h
 *  Author        : Dang Minh Phuong
 *  Email         : kamejoko80@yahoo.com
 *
 *  This program is free software, you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef __FRAME_H__
#define __FRAME_H__

#include <linux/types.h>

/*
 * IPC Frame structure:
 *  _____ _____ ______ ______ ____________ ______________
 * |     |     |      |      |            |              |
 * | SOH | LEN | CRC8 | CRC8 |    DATA    | ZERO PADDING |
 * |_____|_____|______|______|____________|______________|
 *
 * |<----------------- IPC_TRANSFER_LEN ---------------->|
 *
 *  SOH  : u8  : Start of header
 *  LEN  : u16 : Data length (not include data CRC8)
 *  CRC8 : u8  : Header CRC checksum
 *  CRC8 : u8  : Frame data checksum
 *  DATA : u8  : IPC frame data
 *
 */

/*!
 * IPC parameter configuration
 */
#define IPC_TRANSFER_LEN (257)
#define IPC_DATA_MAX_LEN (IPC_TRANSFER_LEN - 5)
#define IPC_FRAME_SOH    (0xA5)

/*!
 * IPC FIFO buffer size
 */
#define FIFO_BUF_SIZE    (IPC_TRANSFER_LEN * 10)

/*!
 * IPC frame header structure
 */
typedef struct ipc_frame_header_s
{
    u8  soh;    /* Start of header     */
    u16 len;    /* Payload length      */
    u8  crc8;   /* Header CRC checksum */
} __attribute__((packed, aligned(1))) ipc_frame_header_t;

/*!
 * IPC frame structure
 */
typedef struct ipc_frame_s
{
    ipc_frame_header_t header; /* Frame header      */
    u8 crc8;                   /* Data CRC checksum */
    u8 data[IPC_DATA_MAX_LEN]; /* Frame data        */
} __attribute__((packed, aligned(1))) ipc_frame_t;



bool ipc_frame_create(ipc_frame_t *frame, u8 *data, u16 len);
void ipc_frame_print(ipc_frame_t *frame);
bool ipc_frame_check(ipc_frame_t *frame);

#endif /* __FRAME_H__ */
