#ifndef BOOTLOADER_H
#define BOOTLOADER_H

/***********includes*************/
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "usart.h"
#include "crc.h"
/***********macro decleration*************/
#define BL_Debug_UART  								&huart2
#define BL_RX_HOST_PACKET          		&huart3
#define BL_CRC_OBJ                    &hcrc
#define BL_ENABLE_UART_DEBUG_MESSAGE  0X00
#define BL_ENABLE_SPI_DEBUG_MESSAGE   0X01
#define BL_ENABLE_CAN_DEBUG_MESSAGE   0X02
#define BL_DEBUG_METHOD  (BL_ENABLE_UART_DEBUG_MESSAGE)

#define BL_HOST_BUFFER_RX_LENGH       200

#define CBL_GET_VER_CMD               0X10
#define CBL_GET_HELP_CMD							0X11
#define CBL_GET_CID_CMD								0X12
#define CBL_GET_RDP_STATUS_CMD		  	0X13
#define CBL_GO_TO_ADDR_CMD					  0X14
#define CBL_FLASH_ERASE_CMD						0X15
#define CBL_MEM_WR_CMD								0X16
#define CBL_EN_R_W_PROTECT_CMD				0X17
#define CBL_MEM_READ_CMD							0X18
#define CBL_READ_SECTOR_STATUS_CMD		0X19
#define CBL_OTP_READ_CMD							0X20
#define CBL_CHNG_RPL_CMD				      0X21
/*CBL_GET_VER_CMD*/
#define CBL_VENDOR_ID                 100
#define CBL_SW_MAJOR_VERSION          1
#define CBL_SW_MINOR_VERSION          0
#define CBL_SW_PATCH_VERSION          0
#define CRC_PASSED										0X00
#define CRC_FAILED                    0X01
#define CBL_SEND_ACK									0X00
#define CBL_SEND_NACK									0X01

#define FLASH_SECTOR2_BASE_ADDRESS    0X08008000

#define ADDRESS_IS_INVALID      			0X00
#define ADDRESS_IS_VALID      		  	0X01
#define STM32F407_SRAM1_SIZE          (112 * 1024)
#define STM32F407_SRAM2_SIZE          (16 * 1024)
#define STM32F407_SRAM3_SIZE          (64 * 1024)
#define STM32F407_FLASH_SIZE          (1024 * 1024)
#define STM32F407_SRAM1_END           (SRAM1_BASE + STM32F407_SRAM1_SIZE)
#define STM32F407_SRAM2_END           (SRAM2_BASE + STM32F407_SRAM2_SIZE)
#define STM32F407_SRAM3_END           (CCMDATARAM_BASE + STM32F407_SRAM3_SIZE)
#define STM32F407_FLASH_END           (FLASH_BASE + STM32F407_FLASH_SIZE)
/*CBL_FLASH_ERASE_CMD*/
#define ERASE_SUCCESSFUL							0X02
#define ERASE_UNSUCCESSFUL						0X03
#define FLASH_SECTOR_MAX_SIZE      		12
#define INVALID_SECTOR_NUMBER         0X00
#define VALID_SECTOR_NUMBER           0X01
#define CBL_FLASH_MASS_ERASE     	    0XFF
#define HAL_SUCCESSFULLY_ERASED       0xFFFFFFFFU
 /*CBL_MEM_WR_CMD*/
#define INVALID_ADDRESS              			  0X00
#define VALID_ADDRESS                			  0X01
#define FLASH_PAYLOAD_WRITE_FAILED    			0X00
#define FLASH_PAYLOAD_WRITE_SUCCESS  				0X01
/*CBL_GET_RDP_STATUS_CMD*/
#define FLASH_READ_PROTECTION_LEVEL_FAILED  0X00
#define FLASH_READ_PROTECTION_LEVEL_PASSED  0X01
/*CBL_CHNG_RPL_CMD*/
#define ROP_LEVEL_CHANGE_INVALID            0X00
#define ROP_LEVEL_CHANGE_VALID              0X01
/***********macro functions decleration*************/

/***********data types decleration*************/
typedef enum {
BL_NACK = 0,
BL_OK	
}BL_STATUS;

typedef void (*jump_ptr)(void);

/********** software interfaces decleration****************/
BL_STATUS BL_UART_Host_Fetch_Command (void);
void BL_print_message (char *format , ...);
#endif  /*BOOTLOADER_H*/