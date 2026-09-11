/*
 * sd_spi.h
 *
 *  Created on: Aug 29, 2024
 *      Author: meowchine1 
 */

#ifndef INC_SD_SPI_H_
#define INC_SD_SPI_H_

#include <stdint.h>

#define ATEMPT_LIMIT 			100
#define BLOCK_SIZE_IN_BYTE 		512
#define BLOCK_SIZE_IN_4xBYTE 	128

/**
 * @brief  SD status structure definition
 */
enum {
	BSP_SD_OK = 0x00, MSD_OK = 0x00, BSP_SD_ERROR = 0x01, BSP_SD_TIMEOUT
};

typedef enum {
	/* R1 answer value */
	INITIAL = (0xAA),
	SD_R1_READY = (0x00),
	SD_R1_IN_IDLE = (0x01),
	SD_R1_ERASE_RESET = (0x02),
	SD_R1_ILLEGAL_CMD = (0x04),
	SD_R1_CRC_ERROR = (0x08),
	SD_R1_ERASE_SEQ_ERROR = (0x10),
	SD_R1_ADDR_ERROR = (0x20),
	SD_R1_PARAM_ERROR = (0x40),
	SD_R1_MSB_1_ERR = 0x80,
	SD_R1_ILLEGAL_RESPONSE = (0xff),
	/* R2 answer value */
	SD_R2_NO_ERROR = 0x00,
	SD_R2_CARD_LOCKED = 0x01,
	SD_R2_LOCKUNLOCK_ERROR = 0x02,
	SD_R2_ERROR = 0x04,
	SD_R2_CC_ERROR = 0x08,
	SD_R2_CARD_ECC_FAILED = 0x10,
	SD_R2_WP_VIOLATION = 0x20,
	SD_R2_ERASE_PARAM = 0x40,
	SD_R2_OUTOFRANGE = 0x80,

	/**
	 * @brief  Data response error
	 */
	SD_DATA_OK = (0x05),
	SD_WRITE_BUSY = (0x00),
	SD_DATA_CRC_ERROR = (0x0B),
	SD_DATA_WRITE_ERROR = (0x0D),
	SD_DATA_TIMEOUT_ERROR = (0xFF)
} SD_Error;

typedef enum {
	VOLTAGE_ACC_27_33,
	VOLTAGE_ACC_LOW,
	VOLTAGE_ACC_RES1,
	VOLTAGE_ACC_RES2,
	VOLTAGE_ND
} Voltage_StatusTypeDef;

typedef enum {
	SD_OK, SD_CMD0_ERR, SD_CMD8_ERR, SD_ACMD41_ERR, SD_UNKNOWN_ERR
} initSD_StatusTypeDef;

typedef struct {
	/* Header part */
	uint8_t CSDStruct :2; /* CSD structure */
	uint8_t Reserved1 :6; /* Reserved */
	uint8_t TAAC :8; /* Data read access-time 1 */
	uint8_t NSAC :8; /* Data read access-time 2 in CLK cycles */
	uint8_t TRAN_SPEED :8; /* Max data transfer rate, 32h or 5Ah */
	uint16_t CardComdClasses :12; /* Card command classes */
	uint8_t RdBlockLen :4; /* Max. read data block length */
	uint8_t PartBlockRead :1; /* Partial blocks for read allowed */
	uint8_t WrBlockMisalign :1; /* Write block misalignment */
	uint8_t RdBlockMisalign :1; /* Read block misalignment */
	uint8_t DSRImpl :1; /* DSR implemented */
	uint8_t Reserved2 :6;
	uint32_t DeviceSize :22;
	uint8_t Reserved3 :1;
	uint8_t EraseSingleBlockEnable :1; /* Erase single block enable */
	uint8_t EraseSectorSize :7; /* Erase group size multiplier */
	uint8_t WrProtectGrSize :7; /* Write protect group size */
	uint8_t WrProtectGrEnable :1; /* Write protect group enable */
	uint8_t Reserved4 :2; /* Reserved */
	uint8_t WrSpeedFact :3; /* Write speed factor */
	uint8_t MaxWrBlockLen :4; /* Max. write data block length */
	uint8_t WriteBlockPartial :1; /* Partial blocks for write allowed */
	uint8_t Reserved5 :5; /* Reserved */
	uint8_t FileFormatGrouop :1; /* File format group */
	uint8_t CopyFlag :1; /* Copy flag (OTP) */
	uint8_t PermWrProtect :1; /* Permanent write protection */
	uint8_t TempWrProtect :1; /* Temporary write protection */
	uint8_t FileFormat :2; /* File Format */
	uint8_t Reserved6 :2; /* Reserved */
	uint8_t crc :7; /* Reserved */
	uint8_t Reserved7 :1; /* always 1*/

} SD_CSD;

typedef struct {
	uint8_t ManufacturerID; /* ManufacturerID */
	uint16_t OEM_AppliID; /* OEM/Application ID */
	uint32_t ProdName1; /* Product Name part1 */
	uint8_t ProdName2; /* Product Name part2*/
	uint8_t ProdRev; /* Product Revision */
	uint32_t ProdSN; /* Product Serial Number */
	uint8_t Reserved1 :4; /* Reserved1 */
	uint16_t ManufactDate :12; /* Manufacturing Date */
	uint8_t CID_CRC :7; /* CID CRC */
	uint8_t Reserved2 :1; /* always 1 */
} SD_CID;

typedef struct {

	uint8_t CardCapacitySt :1;
	uint8_t CardPowerSt :1;

} SD_OCR;

typedef struct {
	SD_Error SD_LastResp;
	Voltage_StatusTypeDef Voltage_st;

	SD_OCR Ocr;
	SD_CSD Csd;
	SD_CID Cid;
	uint32_t CardCapacity; /*!< Card Capacity */
	uint32_t CardBlockSize; /*!< Card Block Size */
	uint32_t LogBlockNbr; /*!< Specifies the Card logical Capacity in blocks   */
	uint32_t LogBlockSize; /*!< Specifies logical block size in bytes           */
	uint8_t Power;  /* Card power status */

} SD_CardInfo;

initSD_StatusTypeDef InitNAND();
uint8_t SD_GetCardState();
SD_Error ReadNandBlock(uint8_t *pData, uint32_t ReadAddr);
SD_Error WriteNandBlock(uint8_t *pData, uint32_t WriteAddr);
SD_Error EraseNand(uint32_t Start, uint32_t End);

void ResetCardPower();

//
//initSD_StatusTypeDef InitSD();
//
//SD_Error BSP_SD_ReadBlock(uint8_t *pData, uint32_t ReadAddr);
//SD_Error BSP_SD_WriteBlock(uint8_t *pData, uint32_t WriteAddr);
//uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr);
//void SD_GetCardInfo();
//uint8_t SD_GetCardState(void);

#endif /* INC_SD_SPI_H_ */
