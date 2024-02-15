/*
 * NRF24L01.c
 *  Created on: Jul 26, 2023
 *      Author: Ahmed Gaber
 */

//To be able to communicate with registers of stm32
#include "stm32f1xx_hal.h"
#include "NRF24L01.h" //NRF registers file

//SPI Handler
extern SPI_HandleTypeDef hspi1; //has the typedef (struct) for SPI
#define NRF24_SPI &hspi1 //address of the struct

// Define chip pin, PORT
#define NRF24_CE_PORT  GPIOA
#define NRF24_CE_PIN  GPIO_PIN_3


#define NRF24_CSN_PORT  GPIOA
#define NRF24_CSN_PIN  GPIO_PIN_4



void CS_Select (void)
{//bring to low to select
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET);
}

void CS_UnSelect (void)
{
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);
}

//it control if we make it work or not
void CE_Enable (void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

void CE_Disable (void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}



// write a single byte to the particular register
void nrf24_WriteReg (uint8_t Reg, uint8_t Data)
{
	uint8_t buf[2];
	buf[0] = Reg|1<<5; //when writing to address 5th bit should be 1  0001AAA
	buf[1] = Data;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, buf, 2, 1000);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}

//write multiple bytes starting from a particular register
void nrf24_WriteRegMulti (uint8_t Reg, uint8_t *data, int size)
{
	uint8_t buf[2];
	buf[0] = Reg|1<<5;


	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, buf, 1, 100);
	HAL_SPI_Transmit(NRF24_SPI, data, size, 1000);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}


uint8_t nrf24_ReadReg (uint8_t Reg)
{
	uint8_t data=0;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);//give name of register you want to read from
	HAL_SPI_Receive(NRF24_SPI, &data, 1, 100); //store it in the data

	// Pull the CS HIGH to release the device
	CS_UnSelect();

	return data;
}


/* Read multiple bytes from the register */
void nrf24_ReadReg_Multi (uint8_t Reg, uint8_t *data, int size)
{
	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, data, size, 1000);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}


// send the command to the NRF
void nrfsendCmd (uint8_t cmd)
{
	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}


void NRF24_Init (void)
{
	// disable the chip before configuring the device
	CE_Disable();

	nrf24_WriteReg(CONFIG, 0);  // will be configured later

	nrf24_WriteReg(EN_AA, 0);  // No Auto ACK

	nrf24_WriteReg (EN_RXADDR, 0);  // Not Enabling any data pipe right now

	nrf24_WriteReg (SETUP_AW, 0x03);  // 5 Bytes for the TX/RX address 0x03=11

	nrf24_WriteReg (SETUP_RETR, 0);   // No retransmission

	nrf24_WriteReg (RF_CH, 0);  // will be setup during Tx or RX

	nrf24_WriteReg (RF_SETUP, 0x0E);   // Power= 0db, data rate = 2Mbps

	// Enable the chip after configuring the device
	CE_Enable();

}


// set up the Tx mode

void NRF24_TxMode (uint8_t *Address, uint8_t channel)
{
	// disable the chip before configuring the device
	CE_Disable();
	uint8_t Test1_RF_CH=nrf24_ReadReg(RF_CH);
	nrf24_WriteReg (RF_CH, channel);  // select the channel
	//test
	uint8_t Test_RF_CH=nrf24_ReadReg(RF_CH);
	nrf24_WriteRegMulti(TX_ADDR, Address, 5);  // Transmit address to NRF by writing in TX_ADDR


	// power up the device
	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1<<1);   // write 1 in the PWR_UP bit
	//config = config & (0xF2);    // write 0 in the PRIM_RX, and 1 in the PWR_UP, and all other bits are masked
	nrf24_WriteReg (CONFIG, config);

	// Enable the chip after configuring the device
	CE_Enable();
}


// transmit the data

uint8_t NRF24_Transmit (uint8_t *data)
{
	uint8_t cmdtosend = 0;
	CS_Select();

	// payload command (payload means prepare that i will dend data data)
	cmdtosend = W_TX_PAYLOAD; //send ack on receving the data in receiver
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

	// send the payload
	HAL_SPI_Transmit(NRF24_SPI, data, 2, 1000);

	// Unselect the device
	CS_UnSelect();

	HAL_Delay(1);

	uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

	// check the fourth bit of FIFO_STATUS to know if the TX fifo is empty (empty means data is transmitted
	// bit 2:3 always by 0 but if device removed they will be 1 but if 0 device is still in contact
	if ((fifostatus&(1<<4)) && (!(fifostatus&(1<<3))))
	{
		//if not empty flush it
		cmdtosend = FLUSH_TX;
		nrfsendCmd(cmdtosend);
		return 1;
	}

	return 0;
}

void NRF24_RxMode (uint8_t *Address, uint8_t channel)
{
	// disable the chip before configuring the device
	CE_Disable();

	//nrf24_reset (STATUS);
	uint8_t test_channel=nrf24_ReadReg(RF_CH);
	nrf24_WriteReg (RF_CH, channel);  // select the channel
	uint8_t test1_channel=nrf24_ReadReg(RF_CH);

	// select data pipe 2
	uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR); //read the datapipe first then modify the bit you want
	en_rxaddr = en_rxaddr | (1<<1);
	nrf24_WriteReg (EN_RXADDR, en_rxaddr);
	uint8_t test1_en_rxaddr = nrf24_ReadReg(EN_RXADDR);

	nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5);  // Write the Pipe1 address

	nrf24_WriteReg (RX_PW_P1, 2);   // 2 bytesize payload  for pipe 1

	// power up the device in Rx mode
	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1<<1) | (1<<0); //set bit 0 in RX
	nrf24_WriteReg (CONFIG, config);
	uint8_t Test_config = nrf24_ReadReg(CONFIG);
	// Enable the chip after configuring the device
	CE_Enable();
}


//check if data avilable at receiver or not
uint8_t isDataAvailable (int pipenum)
{
	//first read status register
	uint8_t status = nrf24_ReadReg(STATUS);

	/*check 1: 6th bit INTR flag if data received
			2: 1th
	*/
	if ((status&(1<<6))&&(status&(pipenum<<1)))
	{
		//clear INTR flag of receiving
		nrf24_WriteReg(STATUS, (1<<6));

		//data received succcefly
		return 1;
	}

	return 0;
}

/*
 * @breif: receive data from data pipe receiver into our buffer
 */
void NRF24_Receive (uint8_t *data)
{
	uint8_t cmdtosend = 0;

	// select the device
	CS_Select();

	// payload command
	//inform NRF we want to receive data from data pipe
	cmdtosend = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

	// Receive the payload
	HAL_SPI_Receive(NRF24_SPI, data, 2, 1000); //2 bit data received

	// Unselect the device
	CS_UnSelect();

	HAL_Delay(1);

	cmdtosend = FLUSH_RX; //once data received flush NRF fifo
	nrfsendCmd(cmdtosend);
}
