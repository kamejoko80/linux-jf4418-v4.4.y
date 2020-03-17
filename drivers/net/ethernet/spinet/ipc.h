#ifndef _IPC_H_
#define _IPC_H_

#include "spinet.h"

/* SPI GPIO Configuration (SPI master)
   Pin 24  ------> SPI0_CSN
   Pin 23  ------> SPI1_SCK
   Pin 21  ------> SPI1_MISO
   pin 19  ------> SPI1_MOSI

   Pin 07  ------> m_status_out
   Pin 15  ------> s_status_in
   Pin 11  ------> m_send_rqst
   Pin 13  ------> m_irq_in
*/

/* SPI GPIO Configuration (SPI slaver)
   PA15     ------> SPI3_NSS
   PC10     ------> SPI3_SCK
   PC11     ------> SPI3_MISO
   PC12     ------> SPI3_MOSI

   PB8      ------> s_status_out
   PB9      ------> m_status_in
   PE0      ------> s_send_rqst (need to pull down by external resistor)
   PB1      ------> s_irq_in
*/

/* Connection
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
#define WAITTIME                   (2)

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

#endif
