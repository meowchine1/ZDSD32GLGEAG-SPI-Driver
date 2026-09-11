/*
 * sd_spi.c
 *
 *  Created on: Aug 29, 2024
 *      Author: meowchine1
 */
#include <logger.h>
#include "zdsd32glgeag.h"
#include <stdio.h>
#include "spi.h"
#include "main.h"
#include "logger.h"
#include "stdlib.h"
#include <string.h>
#include <malloc.h>

static SD_CardInfo NAND;

/**
 * @brief  Start Data tokens:
 *         Tokens (necessary because at nop/idle (and CS active) only 0xff is
 *         on the data/command line)
 */
#define SD_TOKEN_START_DATA_SINGLE_BLOCK_READ    0xFE  /* Data token start byte, Start Single Block Read */
#define SD_TOKEN_START_DATA_MULTIPLE_BLOCK_READ  0xFE  /* Data token start byte, Start Multiple Block Read */
#define SD_TOKEN_START_DATA_SINGLE_BLOCK_WRITE   0xFE  /* Data token start byte, Start Single Block Write */
#define SD_TOKEN_START_DATA_MULTIPLE_BLOCK_WRITE 0xFD  /* Data token start byte, Start Multiple Block Write */
#define SD_TOKEN_STOP_DATA_MULTIPLE_BLOCK_WRITE  0xFD  /* Data toke stop byte, Stop Multiple Block Write */

/*************************************** COMMAND DEFINITIONS **************************************
 * @brief  Commands: CMDxx = CMD-number | 0x40
 *
 */
#define TIMEOUT (5000)

#define BLOCK_SIZE_BYTE		512

#define SD_DUMMY_BYTE       0xFF
#define SD_CMD_LENGTH       6
#define TX_CMD_SIZE 		6			/* TX COMMAND SIZE IN BYTES */
#define CMD0        		(0)  		/* GO_IDLE_STATE */
#define CMD0_CRC    		(0x95)
#define CMD1				(1)			/* SEND_OP_COND (MMC) */
#define CMD2				(2)
#define CMD8				(8)			/* SEND_IF_COND */
#define CMD8_ARG 			(0x1AA)
#define CMD8_CHECK_PATTERN 	(0xAA)
#define CMD8_VHS 			(0b1) 		/* Voltage supplied 0b1 -- 2.7 - 3.6V, 0b10 -- reserved for low voltage range */
#define CMD8_CRC    		(0x86)
#define CMD9				(9)			/* SEND_CSD */
#define CMD10				(10)		/* SEND_CID */
#define CMD12				(12)		/* STOP_TRANSMISSION */
#define CMD13            	(13)  		/* CMD13 = 0x4D */
#define ACMD13				(0x80+13)	/* SD_STATUS (SDC) */
#define CMD16				(16)		/* SET_BLOCK LEN*/
#define CMD17				(17)		/* READ_SINGLE_BLOCK */
#define CMD18				(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23				(23)		/* SET_BLOCK_COUNT (MMC) */
#define	ACMD23				(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24				(24)		/* WRITE_BLOCK */
#define CMD25				(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD27				(27)		/* EDIT CSD */
#define CMD32				(32)		/* ERASE_ER_BLK_START */
#define CMD33				(33)		/* ERASE_ER_BLK_END */
#define CMD38				(38)		/* ERASE */
#define ACMD41  			(41) 		/* SEND_OP_COND*/
#define ACMD41_ARG_HCS  	(0x40000000) 	/* arg with HCS bit setted */
/* ACMD41_ARG DESCRIPTION -------------------------------------------
 * 														 			 |
 * HCS: [30bit] = 0 - SDSC Only Host, = 1 - SDHC or SDXC supported   |
 * 																	 |													*/
#define CMD58   (58)	 	/* READ_OCR */
#define CMD55   (55) 		/* APP_CMD (SEND BEFORE SENDING ACMD COMMAND) */

#define OCR_LEN (4)
#define CSD_LEN (16)
#define CID_LEN (16)
#define R1_LEN  (2)		/* response size in bytes (additional 0xFF byte, specific for this SD)*/
#define R2_LEN  (18) 	/*CID, CSD (R1 2 bytes + 16-byte block read)*/
#define R3_LEN  (7) 	/*OCR*/
#define R7_LEN  (6)    	/* R7 size 5 bytes, but you know that last byte of response is R1.
The SD for which this driver is designed adds FF byte in end of R1 response*/

/* Private function prototypes -----------------------------------------------*/
void setCS(uint8_t state);
void send_command(uint8_t *cmd, uint8_t *rx_bf, uint8_t tx_nb, uint8_t rx_nb);

void sendCMD0(uint8_t *rx_bf);
void sendCMD8(uint8_t *rx_bf);
void sendACMD41(uint8_t *rx_bf);

void parse_R1(const uint8_t res);
void parse_R7(const uint8_t *res);
void parse_OCR(const uint8_t *res);
void parse_CID(const uint8_t *res);

void read_CSD();
void read_CID();
void read_CSD();

uint8_t SD_GetCSDRegister(SD_CSD *Csd);
uint8_t SD_GetCIDRegister(SD_CID *Cid);
void SD_SendCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t *rx_bf,
		uint8_t rx_len);

initSD_StatusTypeDef workInit();
void wakeUpSPI(void);
static uint8_t SD_WaitData(uint8_t data);
void WriteReadData(uint8_t *ptr, uint8_t *pData, uint16_t BlockSize);
uint8_t SendByte(uint8_t byte);

void SD_GetCardInfo();
SD_Error BSP_SD_ReadBlock(uint8_t *pData, uint32_t addr);
SD_Error BSP_SD_WriteBlock(uint8_t *pData, uint32_t addr);
uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr);

initSD_StatusTypeDef InitSD();

void TurnNandOn();
void TurnNandOff();

/*Public functions -----------------------------------------------------------*/

void ResetCardPower() {
	TurnNandOff();
	HAL_Delay(1500);
	TurnNandOn();
}

initSD_StatusTypeDef InitNAND() {

	char data[100];
	uint8_t atempt_limit = 0;
	initSD_StatusTypeDef initSDResult = InitSD();
	while ((initSDResult != SD_OK) & (atempt_limit < ATEMPT_LIMIT)) {
		initSDResult = InitSD();
		atempt_limit++;
	}

	if (initSDResult != SD_OK) {
		uint8_t len = sprintf(data, "\n\rNAND is not init, state = %d",
				(uint8_t) initSDResult);
		log(data, len);
	}

	return initSDResult;
}

SD_Error SD_GetCardState() {
	uint8_t rx_bf[20] = { 0 };
	setCS(GPIO_PIN_RESET);
	/* Send CMD13 (SD_SEND_STATUS) to get SD status */
	SD_SendCmd(CMD13, 0, 0xFF, rx_bf, R1_LEN);
	setCS(GPIO_PIN_SET);
	return rx_bf[1];
}

SD_Error ReadNandBlock(uint8_t *pData, uint32_t ReadAddr) {
	return BSP_SD_ReadBlock(pData, ReadAddr);
}

SD_Error WriteNandBlock(uint8_t *pData, uint32_t WriteAddr) {

	return BSP_SD_WriteBlock(pData, WriteAddr);
}

SD_Error EraseNand(uint32_t Start, uint32_t End) {
	return BSP_SD_Erase(Start, End);
}

/* Private functions definitions ----------------------------------------------*/

void TurnNandOn() {
	HAL_GPIO_WritePin(SD_V_GPIO_Port, SD_V_Pin, 0);
	NAND.Power = 1;
}

void TurnNandOff() {
	HAL_GPIO_WritePin(SD_V_GPIO_Port, SD_V_Pin, 1);
	NAND.Power = 0;
}

void setCS(uint8_t state) {
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, state);
}

void send_command(uint8_t *cmd, uint8_t *rx_bf, uint8_t tx_nb, uint8_t rx_nb) {
	setCS(GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, cmd, rx_bf, tx_nb, TIMEOUT);
	HAL_SPI_Receive(&hspi1, rx_bf, rx_nb, TIMEOUT);
	setCS(GPIO_PIN_SET);
}

void SD_SendCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t *rx_bf,
		uint8_t rx_len) {
	uint8_t tx_frame[SD_CMD_LENGTH] = { 0 };

	/* Prepare Frame to send */
	tx_frame[0] = (Cmd | 0x40);
	tx_frame[1] = (uint8_t) (Arg >> 24);
	tx_frame[2] = (uint8_t) (Arg >> 16);
	tx_frame[3] = (uint8_t) (Arg >> 8);
	tx_frame[4] = (uint8_t) (Arg);
	tx_frame[5] = (Crc | 0x01);

	/* Send the command */
	HAL_SPI_TransmitReceive(&hspi1, tx_frame, rx_bf, SD_CMD_LENGTH,
	TIMEOUT);
	HAL_SPI_Receive(&hspi1, rx_bf, rx_len, TIMEOUT);
}

void WriteDataBlock(uint8_t *pTxData, uint16_t size) {
	HAL_SPI_Transmit(&hspi1, pTxData, size, TIMEOUT);
}

uint8_t SpiTransmitReceiveByte(uint8_t byte) {
	uint8_t receive = 0;
	HAL_SPI_TransmitReceive(&hspi1, &byte, &receive, 1, TIMEOUT);
	return receive;
}

uint8_t SendByte(uint8_t byte) {
	HAL_SPI_Transmit(&hspi1, &byte, 1, TIMEOUT);
	return 0;
}

void wakeUpSPI() {
	/* SD needs small delay after power reset*/
	HAL_Delay(1);
	uint8_t args_init[10] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF };
	setCS(GPIO_PIN_SET);
	HAL_SPI_Transmit(&hspi1, args_init, 10, TIMEOUT);/*
	 *	76 clock to wake up SD
	 *! CS and MOSI must be 1
	 * */
}

/**
 * @brief  Waits a data from the SD card
 * @param  data : Expected data from the SD card
 * @retval BSP_SD_OK or BSP_SD_TIMEOUT
 */
uint8_t SD_WaitData(uint8_t data) {
	uint16_t timeout = 0xFFFF;
	uint8_t readvalue;
	/* Check if response is got or a timeout is happen */
	do {
		readvalue = SendByte(SD_DUMMY_BYTE);
		timeout--;
	} while ((readvalue != data) && timeout);

	if (timeout == 0) {
		/* After time out */
		return BSP_SD_TIMEOUT;
	}

	/* Right response got */
	return BSP_SD_OK;
}

/* ********************************R1 reading, parsing****************************************** */
void parse_R1(const uint8_t res) {
	char *err = "\n\rParsing R1 ERR:";
	char *ok = "\n\rParsing R1 OK:";
	char data[50];
	uint8_t len = 0;
	if (SD_R1_READY == res) {
		len = sprintf(data, "%s CARD IS READY", ok);
		NAND.SD_LastResp = SD_R1_READY;

	} else if (SD_R1_IN_IDLE == res) {
		len = sprintf(data, "%s IN IDLE", ok);
		NAND.SD_LastResp = SD_R1_IN_IDLE;

	} else if (SD_R1_MSB_1_ERR == res) {
		len = sprintf(data, "%s MSB 1 ERR", err);
		NAND.SD_LastResp = SD_R1_MSB_1_ERR;

	} else if (SD_R1_PARAM_ERROR == res) {
		len = sprintf(data, "%s PARAM ERROR", err);
		NAND.SD_LastResp = SD_R1_PARAM_ERROR;

	} else if (SD_R1_ADDR_ERROR == res) {
		len = sprintf(data, "%s ADDR ERROR", err);
		NAND.SD_LastResp = SD_R1_ADDR_ERROR;

	} else if (SD_R1_ERASE_SEQ_ERROR == res) {
		len = sprintf(data, "%s ERASE SEQ ERROR", err);
		NAND.SD_LastResp = SD_R1_ERASE_SEQ_ERROR;

	} else if (SD_R1_CRC_ERROR == res) {
		len = sprintf(data, "%s CRC ERROR", err);
		NAND.SD_LastResp = SD_R1_CRC_ERROR;

	} else if (SD_R1_ILLEGAL_CMD == res) {
		len = sprintf(data, "%s MSB_1_ERR", err);
		NAND.SD_LastResp = SD_R1_ILLEGAL_CMD;

	} else if (SD_R1_ERASE_RESET == res) {
		len = sprintf(data, "%s ERASE RESET", err);
		NAND.SD_LastResp = SD_R1_ERASE_RESET;

	} else {
		NAND.SD_LastResp = SD_R1_ILLEGAL_RESPONSE;
		len = sprintf(data, "%s ILLEGAL RESPONSE", err);
	}

	log(data, len);
}

/* ********************************R7 reading, parsing****************************************** */
#define CMD_VER(X)         		(X >> 4)
#define VOLTAGE_ACC_27_33_Def   0b1
#define VOLTAGE_ACC_LOW_Def     0b10
#define VOLTAGE_ACC_RES1_Def    0b100
#define VOLTAGE_ACC_RES2_Def    0b1000

void parse_R7(const uint8_t *res) {
	char data[150];
	uint8_t len = 0;

	parse_R1(res[1]);

	if ((NAND.SD_LastResp == SD_R1_IN_IDLE)
			|| (NAND.SD_LastResp == SD_R1_READY)) {
		len = sprintf(data, "%.*s \n\rCommand Version: %d", len, data,
				(uint8_t) CMD_VER(res[2]));
		len = sprintf(data, "%.*s \n\rVoltage Accepted:", len, data);

		if (res[4] == VOLTAGE_ACC_27_33_Def) {
			NAND.Voltage_st = VOLTAGE_ACC_27_33;
			len = sprintf(data, "%.*s 2.7-3.6V", len, data);

		} else if (res[4] == VOLTAGE_ACC_LOW_Def) {
			NAND.Voltage_st = VOLTAGE_ACC_LOW;
			len = sprintf(data, "%.*s LOW VOLTAGE", len, data);

		} else if (res[4] == VOLTAGE_ACC_RES1_Def) {
			NAND.Voltage_st = VOLTAGE_ACC_RES1;
			len = sprintf(data, "%.*s RESERVED", len, data);

		} else if (res[4] == VOLTAGE_ACC_RES2_Def) {
			NAND.Voltage_st = VOLTAGE_ACC_RES2;
			len = sprintf(data, "%.*s RESERVED", len, data);

		} else {
			NAND.Voltage_st = VOLTAGE_ND;
			len = sprintf(data, "%.*s NOT DEFINED", len, data);
		}

	} else {
		len = sprintf(data, "%.*s R1 error", len, data);
	}

	char *chk_ptr_res =
	CMD8_CHECK_PATTERN == res[5] ?
			"\n\rCheck pattern conforms" : "\n\rCheck pattern MISSMATCHES";
	len = sprintf(data, "\n\rR7: \n\r %.*s %s", len, data, chk_ptr_res);
	log(data, len);
}

/* ******************************** OCR reading, parsing****************************************** */
#define POWER_STATUS_BIT 	0x80
#define CARD_CAPACITY_BIT 	0x40
#define VDD_2728(X)         X & 0b10000000
#define VDD_2829(X)         X & 0b00000001
#define VDD_2930(X)         X & 0b00000010
#define VDD_3031(X)         X & 0b00000100
#define VDD_3132(X)         X & 0b00001000
#define VDD_3233(X)         X & 0b00010000
#define VDD_3334(X)         X & 0b00100000
#define VDD_3435(X)         X & 0b01000000
#define VDD_3536(X)         X & 0b10000000

void read_OCR() {
	uint8_t rx_bf[20] = { 0 };
	SD_SendCmd(CMD58, 0x0, 0x0, rx_bf, R1_LEN + OCR_LEN);
	char data[200];
	uint8_t len = 0;
	parse_R1(rx_bf[1]);

	if ((NAND.SD_LastResp == SD_R1_IN_IDLE)
			|| (NAND.SD_LastResp == SD_R1_READY)) {
		len = sprintf(data, "%.*s \n\rCard Power Up Status:", len, data); // 24

		NAND.Ocr.CardPowerSt = (rx_bf[2] & POWER_STATUS_BIT) >> 7;

		if (NAND.Ocr.CardPowerSt) { /* This bit is set to HIGH if the card has finished the power up routine. */

			NAND.Ocr.CardCapacitySt = (rx_bf[2] & CARD_CAPACITY_BIT) >> 6;

			len = sprintf(data, "%.*s READY \tCCS Status: %d", len, data,
					NAND.Ocr.CardCapacitySt);
		} else {
			NAND.Ocr.CardCapacitySt = 0;
			len = sprintf(data,
					"%.*s Card has not finished the power up routine.", len,
					data);
		}
		len = sprintf(data, "%.*s \n\rVDD Window:", len, data);
		if (VDD_2728(rx_bf[3])) {
			/*TODO Does this information need somewhere?*/
			len = sprintf(data, "%.*s 2.7-2.8, ", len, data);
		}
		if (VDD_2829(rx_bf[3])) {
			len = sprintf(data, "%.*s 2.8-2.9, ", len, data);
		}
		if (VDD_2930(rx_bf[3])) {
			len = sprintf(data, "%.*s 2.9-3.0, ", len, data);
		}
		if (VDD_3031(rx_bf[3])) {
			len = sprintf(data, "%.*s 3.0-3.1, ", len, data);
		}
		if (VDD_3132(rx_bf[3])) {
			len = sprintf(data, "%.*s 3.1-3.2, ", len, data);
		}
		if (VDD_3233(rx_bf[3])) {
			len = sprintf(data, "%.*s 3.2-3.3, ", len, data);
		}
		if (VDD_3334(rx_bf[3])) {
			len = sprintf(data, "%.*s 3.3-3.4, ", len, data);
		}
		if (VDD_3435(rx_bf[3])) {
			len = sprintf(data, "%.*s 3.4-3.5, ", len, data);
		}
		if (VDD_3536(rx_bf[3])) {

			len = sprintf(data, "%.*s 3.5-3.6, ", len, data);
		}
	} else {
		len = sprintf(data, "%.*s R1 error", len, data);
	}
	log(data, len);
}

/* ******************************** CID reading, parsing****************************************** */
void read_CID() { /* CMD 2, 10 */
	uint8_t rx_bf[20] = { 0 };
	uint8_t offset = 2;
	SD_SendCmd(CMD10, 0x0, 0x0, rx_bf, R1_LEN + CID_LEN);

	parse_R1(rx_bf[1]);
	if (NAND.SD_LastResp == SD_R1_READY) {
		/* Byte 0 */
		NAND.Cid.ManufacturerID = rx_bf[0 + offset]; // CHECK FIRS RESPONSE BYTE!

		/* Byte 1 */
		NAND.Cid.OEM_AppliID = rx_bf[1 + offset] << 8;

		/* Byte 2 */
		NAND.Cid.OEM_AppliID |= rx_bf[2 + offset];

		/* Byte 3 */
		NAND.Cid.ProdName1 = rx_bf[3 + offset] << 24;

		/* Byte 4 */
		NAND.Cid.ProdName1 |= rx_bf[4 + offset] << 16;

		/* Byte 5 */
		NAND.Cid.ProdName1 |= rx_bf[5 + offset] << 8;

		/* Byte 6 */
		NAND.Cid.ProdName1 |= rx_bf[6 + offset];

		/* Byte 7 */
		NAND.Cid.ProdName2 = rx_bf[7 + offset];

		/* Byte 8 */
		NAND.Cid.ProdRev = rx_bf[8 + offset];

		/* Byte 9 */
		NAND.Cid.ProdSN = rx_bf[9 + offset] << 24;

		/* Byte 10 */
		NAND.Cid.ProdSN |= rx_bf[10 + offset] << 16;

		/* Byte 11 */
		NAND.Cid.ProdSN |= rx_bf[11 + offset] << 8;

		/* Byte 12 */
		NAND.Cid.ProdSN |= rx_bf[12 + offset];

		/* Byte 13 */
		NAND.Cid.Reserved1 |= (rx_bf[13 + offset] & 0xF0) >> 4;
		NAND.Cid.ManufactDate = (rx_bf[13 + offset] & 0x0F) << 8;

		/* Byte 14 */
		NAND.Cid.ManufactDate |= rx_bf[14 + offset];

		/* Byte 15 */
		NAND.Cid.CID_CRC = (rx_bf[15 + offset] & 0xFE) >> 1;
		NAND.Cid.Reserved2 = 1;
	}

}
/* ******************************** CSD reading, parsing****************************************** */

void read_CSD() { /* CMD9 */
	uint8_t rx_bf[20] = { 0 };
	uint8_t offset = 2;

	//SD_SendCmd(CMD9, 0x0, 0x0, rx_bf, R1_LEN + CSD_LEN);
	SD_SendCmd(CMD9, 0x0, 0xFF, rx_bf, R1_LEN + CSD_LEN + 2); /* this SD card adds 2 external bytes after R1 response*/

	parse_R1(rx_bf[1]);
	uint8_t attempt_limit = 0;
	while ((NAND.SD_LastResp != SD_R1_READY) & (attempt_limit < ATEMPT_LIMIT)) {
		SD_SendCmd(CMD9, 0x0, 0xFF, rx_bf, R1_LEN + CSD_LEN + 2); /* this SD card adds 2 external bytes after R1 response*/
		parse_R1(rx_bf[1]);
	}

	if (NAND.SD_LastResp == SD_R1_READY) {
		/* Byte 0 */
		NAND.Csd.CSDStruct = (rx_bf[2 + offset] & 0xC0) >> 6;
		NAND.Csd.Reserved1 = rx_bf[2 + offset] & 0x3F;

		/* Byte 1 */
		NAND.Csd.TAAC = rx_bf[3 + offset];

		/* Byte 2 */
		NAND.Csd.NSAC = rx_bf[4 + offset];

		/* Byte 3 */
		NAND.Csd.TRAN_SPEED = rx_bf[5 + offset];

		/* Byte 4/5 */
		NAND.Csd.CardComdClasses = (rx_bf[6 + offset] << 4)
				| ((rx_bf[7 + offset] & 0xF0) >> 4);
		NAND.Csd.RdBlockLen = rx_bf[7 + offset] & 0x0F;

		/* Byte 6 */
		NAND.Csd.PartBlockRead = (rx_bf[8 + offset] & 0x80) >> 7;
		NAND.Csd.WrBlockMisalign = (rx_bf[8 + offset] & 0x40) >> 6;
		NAND.Csd.RdBlockMisalign = (rx_bf[8 + offset] & 0x20) >> 5;
		NAND.Csd.DSRImpl = (rx_bf[8 + offset] & 0x10) >> 4;

		/* Byte 6, 7 */
		NAND.Csd.Reserved2 = ((rx_bf[8 + offset] & 0x0F) << 2)
				| ((rx_bf[9 + offset] & 0xC0) >> 6);
		/* Byte 7, 8 */
		NAND.Csd.DeviceSize = ((rx_bf[9 + offset] & 0x3F) << 16)
				| (rx_bf[10 + offset] << 8) | rx_bf[11];
		NAND.Csd.Reserved3 = ((rx_bf[12 + offset] & 0x80) >> 7); //change 8 -> 7

		/*Byte 8, 9*/
		NAND.Csd.EraseSingleBlockEnable = (rx_bf[12 + offset] & 0x40) >> 6;
		NAND.Csd.EraseSectorSize = ((rx_bf[12 + offset] & 0x3F) << 1)
				| ((rx_bf[13 + offset] & 0x80) >> 7);

		/*Byte 9*/
		NAND.Csd.WrProtectGrSize = (rx_bf[13 + offset] & 0x7F);

		/* Byte 10 */
		NAND.Csd.WrProtectGrEnable = (rx_bf[14 + offset] & 0x80) >> 7;
		NAND.Csd.Reserved4 = (rx_bf[14 + offset] & 0x60) >> 5;
		NAND.Csd.WrSpeedFact = (rx_bf[14 + offset] & 0x1C) >> 2;

		/*2 last bit Byte 10, 2 first bit Byte 11*/
		NAND.Csd.MaxWrBlockLen = ((rx_bf[14 + offset] & 0x03) << 2)
				| ((rx_bf[15 + offset] & 0xC0) >> 6);
		/*Byte 10 */
		NAND.Csd.WriteBlockPartial = (rx_bf[15 + offset] & 0x20) >> 5;
		NAND.Csd.Reserved5 = (rx_bf[15 + offset] & 0x1F);

		NAND.Csd.FileFormatGrouop = (rx_bf[16 + offset] & 0x80) >> 7;
		NAND.Csd.CopyFlag = (rx_bf[16 + offset] & 0x40) >> 6;
		NAND.Csd.PermWrProtect = (rx_bf[16 + offset] & 0x20) >> 5;
		NAND.Csd.TempWrProtect = (rx_bf[16 + offset] & 0x10) >> 4;
		NAND.Csd.FileFormat = (rx_bf[16 + offset] & 0x0C) >> 2;
		NAND.Csd.Reserved6 = (rx_bf[16 + offset] & 0x03);
		NAND.Csd.crc = (rx_bf[17 + offset] & 0xFE) >> 1;
		NAND.Csd.Reserved7 = 1;
	}

}

void sendCMD8(uint8_t *rx_bf) {
	SD_SendCmd(CMD8, CMD8_ARG, CMD8_CRC, rx_bf, R7_LEN);
	parse_R1(rx_bf[1]);
	parse_R7(rx_bf);
}

void sendCMD0(uint8_t *rx_bf) {
	SD_SendCmd(CMD0, 0x0, CMD0_CRC, rx_bf, R1_LEN);
	parse_R1(rx_bf[1]);
}

void sendACMD41(uint8_t *rx_bf) {
	SD_SendCmd(CMD55, 0x0, 0x0, rx_bf, R1_LEN);
	SD_SendCmd(ACMD41, ACMD41_ARG_HCS, 0x0, rx_bf, R1_LEN);
	parse_R1(rx_bf[1]);
}

initSD_StatusTypeDef InitSD() {
	ResetCardPower();
	char data[150];
	uint8_t attempt_limit = 0;
	uint8_t len = 0;
	uint8_t rx_bf[20] = { 0 };

	wakeUpSPI();
	setCS(GPIO_PIN_RESET);

	sendCMD0(rx_bf);
	attempt_limit = 0;
	while ((NAND.SD_LastResp != SD_R1_IN_IDLE) & (attempt_limit < ATEMPT_LIMIT)) {
		sendCMD0(rx_bf);
		attempt_limit++;
	}
	len = sprintf(data, "\n\rCMD0 attempt = %d", attempt_limit);
	log(data, len);

	if (NAND.SD_LastResp == SD_R1_IN_IDLE) {
		attempt_limit = 0;
		sendCMD8(rx_bf);
		while ((NAND.SD_LastResp != SD_R1_IN_IDLE)
				& (attempt_limit < ATEMPT_LIMIT)) {
			sendCMD8(rx_bf);
			attempt_limit++;
		}
		len = sprintf(data, "\n\rCMD8 attempt = %d", attempt_limit);
		log(data, len);

		if (NAND.SD_LastResp != SD_R1_IN_IDLE) {
			return SD_CMD8_ERR;
		}

		SendByte(SD_DUMMY_BYTE);
		attempt_limit = 0;
		while ((NAND.SD_LastResp != SD_R1_READY)
				& (attempt_limit < ATEMPT_LIMIT)) {

			sendACMD41(rx_bf);
			attempt_limit++;
		}

		len = sprintf(data, "\n\rACMD41 attempt = %d", attempt_limit);
		log(data, len);
		if ((NAND.SD_LastResp != SD_R1_READY)) {
			len = sprintf(data, "\n\rACMD41 FAIL: SD STAYS IN IDLE STATE");
			log(data, len);
			return SD_ACMD41_ERR;
		}

	} else {

		len = sprintf(data,
				"n\rExpected card moves to IDLE state, but card is not");
		log(data, len);
		return SD_CMD0_ERR;
	}
	SD_GetCardInfo();
	setCS(GPIO_PIN_SET);
	return SD_OK;
}

/**
 * @brief  Fills information about specific card.
 * @param  pSD: Pointer to a SD_CardInfo structure that contains all SD
 *         card information.
 */
void SD_GetCardInfo() {

	read_CSD();
	read_OCR();
	NAND.LogBlockSize = 512;
	NAND.CardBlockSize = 512;
	NAND.CardCapacity = (NAND.Csd.DeviceSize + 1) * 1024 * NAND.LogBlockSize;
	NAND.LogBlockNbr = (NAND.CardCapacity) / (NAND.LogBlockSize);
}

/* Reading data from card */
/* Токены ошибки чтения */
#define SD_TOKEN_OOR(X)     X & 0b00001000
#define SD_TOKEN_CECC(X)    X & 0b00000100
#define SD_TOKEN_CC(X)      X & 0b00000010
#define SD_TOKEN_ERROR(X)   X & 0b00000001
#define SD_MAX_READ_ATTEMPTS    1563
SD_Error BSP_SD_ReadBlock(uint8_t *pData, uint32_t addr) {

	uint8_t rx_bf[20] = { 0 };

	setCS(GPIO_PIN_RESET);

	/* Send CMD17 (SD_CMD_READ_SINGLE_BLOCK) to read one block */
	/* Check if the SD acknowledged the read block command: R1 response (0x00: no errors) */
	SD_SendCmd(CMD17, addr, 0xFF, rx_bf, R1_LEN);
	parse_R1(rx_bf[1]);
	if (NAND.SD_LastResp != SD_R1_READY) {
		return NAND.SD_LastResp;
	}

	uint32_t attemptsLimit = 0;
	uint8_t response = 0;
	// wait for a response token (timeout = 100ms)

	response = SpiTransmitReceiveByte(0xFF);
	while ((response == 0xFF) & (++attemptsLimit < SD_MAX_READ_ATTEMPTS)) {
		response = SpiTransmitReceiveByte(0xFF);
	}

	uint16_t crc;
	// if response token is 0xFE
	if (response == 0xFE) {
		// read 512 byte block
		for (uint16_t i = 0; i < 512; i++) {
			pData[i] = SpiTransmitReceiveByte(0xFF);
		}
		// read 16-bit CRC
		crc = crc | (SpiTransmitReceiveByte(0xFF) << 8)
				| SpiTransmitReceiveByte(0xFF);
	} else {

		return response;
	}

	setCS(GPIO_PIN_SET);
	return SD_DATA_OK;
}

/**
 * @brief  Writes block(s) to a specified address in the SD card, in polling mode.
 * @param  pData: Pointer to the buffer that will contain the data to transmit
 * @param  WriteAddr: Address from where data is to be written. The address is counted
 *                   in blocks of 512bytes
 * @param  NumOfBlocks: Number of SD blocks to write
 * @param  Timeout: This parameter is used for compatibility with BSP implementation
 * @retval SD status
 */
/* Put CRC bytes (not really needed by us, but required by SD) */
/*******************************************************************************
 Write single 512 byte block
 token = 0x00 - busy timeout
 token = 0x05 - data accepted
 token = 0xFF - response timeout
 *******************************************************************************/
#define SD_MAX_WRITE_ATTEMPTS   3907

SD_Error BSP_SD_WriteBlock(uint8_t *pData, uint32_t addr) {

	uint8_t rx_bf[20] = { 0 };
	uint32_t attemptsLimit;

	setCS(GPIO_PIN_RESET);

	/* Send CMD24 (SD_CMD_WRITE_SINGLE_BLOCK) to write blocks  and
	 Check if the SD acknowledged the write block command: R1 response (0x00: no errors) */
	SD_SendCmd(CMD24, addr, 0xFF, rx_bf, R1_LEN);
	parse_R1(rx_bf[1]);
	if (NAND.SD_LastResp != SD_R1_READY) {
		return NAND.SD_LastResp;
	}

	/* Send dummy byte for NWR timing : one byte between CMDWRITE and TOKEN */
	SendByte(SD_DUMMY_BYTE);

	/* Send the data token to signify the start of the data */
	SendByte(SD_TOKEN_START_DATA_SINGLE_BLOCK_WRITE);

	/* Write the block data to SD */
	WriteDataBlock(pData, BLOCK_SIZE_IN_BYTE);

	// wait for a response (timeout = 250ms)

	uint8_t writeStatus = SpiTransmitReceiveByte(0xFF);
	attemptsLimit = 0;
	while ((writeStatus == 0xFF) & (++attemptsLimit < SD_MAX_WRITE_ATTEMPTS)) {
		writeStatus = SpiTransmitReceiveByte(0xFF);
	}

	/*
	 * Помните, что токены принятых данных — это 0bxxx00101,
	 * поэтому мы маскируем верхние три бита первого не 0xFF ответа,
	 * который мы получаем, и смотрим, равен ли он 0b00000101.
	 * */
	if ((writeStatus & 0x1F) == SD_DATA_OK) {
		// wait for write to finish (timeout = 250ms). MISO should be LOW (busy)
		uint8_t busy_status = SpiTransmitReceiveByte(0xFF);
		attemptsLimit = 0;
		while ((busy_status == SD_WRITE_BUSY)
				& (++attemptsLimit < SD_MAX_WRITE_ATTEMPTS)) {
			busy_status = SpiTransmitReceiveByte(0xFF);
		}
	}

	setCS(GPIO_PIN_SET);
	return (writeStatus & 0x1F);
}

/**
 * @brief  Erases the specified memory area of the given SD card.
 * @param  StartAddr: Start address in Blocks (Size of a block is 512bytes)
 * @param  EndAddr: End address in Blocks (Size of a block is 512bytes)
 * @retval SD status
 */
uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr) {
	uint8_t retr = SD_DATA_TIMEOUT_ERROR;
	uint8_t rx_bf[20] = { 0 };
	uint8_t attemptsLimit;
	char data[150];
	uint8_t len;

	setCS(GPIO_PIN_RESET);

	/* Send CMD32 (Erase group start) and check if the SD acknowledged the erase command: R1 response (0x00: no errors) */
	SD_SendCmd(CMD32, (StartAddr) * 1, 0xFF, rx_bf, R1_LEN);
	parse_R1(rx_bf[1]);

	attemptsLimit = 0;
	while ((NAND.SD_LastResp != SD_R1_READY) & (++attemptsLimit < ATEMPT_LIMIT)) {
		SD_SendCmd(CMD32, (StartAddr) * 1, 0xFF, rx_bf, R1_LEN);
		parse_R1(rx_bf[1]);
	}

	len = sprintf(data, "\n\CMD32 attempt = %d", attemptsLimit);
	log(data, len);
	if (NAND.SD_LastResp == SD_R1_READY) {

		SendByte(SD_DUMMY_BYTE);

		/* Send CMD33 (Erase group end) and Check if the SD
		 *  acknowledged the erase command: R1 response (0x00: no errors) */
		SD_SendCmd(CMD33, EndAddr, 0xFF, rx_bf, R1_LEN);
		parse_R1(rx_bf[1]);

		if (NAND.SD_LastResp == SD_R1_READY) {
			/* Send CMD38 (Erase) and Check if the SD acknowledged the
			 * erase command: R1 response (0x00: no errors) */
			SD_SendCmd(CMD38, 0, 0xFF, rx_bf, R1_LEN);
			parse_R1(rx_bf[1]);

			uint8_t busy_status = SpiTransmitReceiveByte(0xFF);
			attemptsLimit = 0;

			while ((busy_status != 0xFF)
					& (++attemptsLimit < SD_MAX_WRITE_ATTEMPTS)) {
				busy_status = SpiTransmitReceiveByte(0xFF);
			}

			if (NAND.SD_LastResp == SD_R1_READY) {
				retr = SD_DATA_OK;
				goto exit_proc;
			} else {
				retr = NAND.SD_LastResp;
				goto exit_proc;
			}
		} else {
			retr = NAND.SD_LastResp;
			goto exit_proc;
		}
	} else {
		retr = NAND.SD_LastResp;
		goto exit_proc;
	}

	exit_proc: setCS(GPIO_PIN_SET);
	return retr;
}
