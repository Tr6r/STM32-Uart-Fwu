#ifndef __HOST_H__
#define __HOST_H__

#include <stdint.h>

#define __AK_PACKETED	__attribute__((__packed__))
#define __AK_WEAK		__attribute__((__weak__))

#define SYS_BOOT_CMD_NONE					0x01
#define SYS_BOOT_CMD_UPDATE_REQ				0x02
#define SYS_BOOT_CMD_UPDATE_RES				0x03

#define SYS_BOOT_CONTAINER_EXTERNAL_FLASH	0x02
#define SYS_BOOT_CONTAINER_INTERNAL_FLASH	0x03

#define FIRMWARE_PSK						0x1A2B3C4D /* magic number */

#define UART_BOOT_CMD_HANDSHAKE_REQ			(0x01)
#define UART_BOOT_CMD_HANDSHAKE_RES			(0x02)

#define UART_BOOT_CMD_UPDATE_REQ			(0x03)
#define UART_BOOT_CMD_UPDATE_RES			(0x04)
#define UART_BOOT_CMD_TRANSFER_FW_REQ		(0x05)

#define UART_BOOT_CMD_TRANSFER_FW_RES		(0x06)
#define UART_BOOT_CMD_CHECKSUM_FW_REQ		(0x07)
#define UART_BOOT_CMD_CHECKSUM_FW_RES		(0x08)
#define UART_BOOT_FRAME_DATA_SIZE			(254)
#define UART_BOOT_CMD_DATA_SIZE				(254)
#define CHUNK_SIZE 251

#define APP_START 0x08003000U

#define MAX_RETRY 3

/* ACK command */
#define UART_BOOT_FWU_READY			(0x03)
#define UART_BOOT_FWU_NACK			(0x04)
#define UART_BOOT_FWU_ACK			(0x06)


// State machine for UART boot
enum Uart_Boot_Send_State {
	NO_SEND = 0x00,
	SEND_HANDSACKE_STATE = 0x01,
	SEND_META_STATE = 0x02,
	SEND_DATA_STATE = 0x03,
	SEND_FCS_STATE = 0x04
};

typedef struct {
	uint8_t type;
	uint8_t src_task_id;
	uint8_t des_task_id;
	uint8_t sig;
	uint8_t if_src_type;
	uint8_t if_des_type;
} __AK_PACKETED ak_msg_host_res_t;
typedef struct {
	uint8_t cmd;
	uint8_t subcmd;
} uart_boot_cmd_t;
typedef struct {
	uart_boot_cmd_t boot_cmd;
	uint8_t len;
	uint8_t data[UART_BOOT_CMD_DATA_SIZE];
} uart_boot_data_cmd_t;
typedef struct
{

    uint8_t cmd; /* none, update request, verify request ... */
    uint8_t container; /* external FLASH, EPPROM or directly via io driver... */
    uint8_t io_driver; /* SPI, UART, ... */
    uint32_t des_addr; /* start destination address */
    uint32_t src_addr; /* start source address */
	ak_msg_host_res_t ak_msg_res; /* host message response when update completed */

} firmware_boot_cmd_t;
typedef struct {
	uint32_t psk;
	uint32_t bin_len;
	uint16_t checksum;
} firmware_header_t;


#endif //__HOST_H__
