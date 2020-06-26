/*
 * AK4588EN.c
 *
 * Created: 03-May-18 22:35:50
 *  Author: Willem van der Kooij
 */ 

#include "sam.h"

#include "TWI.h"
#include "UART.h"
#include "AK4588EN.h"

uint8_t AK4588EN_TWI_ReadReg(uint8_t regAddr)
{
	uint32_t timeout = 0xFFFFFF;
	
	TWI0 -> TWI_MMR = TWI_MMR_DADR(AK4588EN_I2C_ADDR);
	TWI0 -> TWI_THR = regAddr;
	
	while ((TWI0 -> TWI_SR & TWI_SR_NACK) && timeout){
		timeout--;
	}
	if (!timeout) return 0;	
	
	
	
	TWI0 -> TWI_MMR = TWI_MMR_MREAD | TWI_MMR_DADR(AK4588EN_I2C_ADDR);
	TWI0 -> TWI_CR = TWI_CR_START | TWI_CR_STOP;
	
	timeout = 0xFFFFFF;
	while ((TWI0 -> TWI_SR & TWI_SR_RXRDY) && timeout){
		timeout--;
	}
	if (!timeout) return 0;
	
	TWI0 -> TWI_CR = TWI_CR_STOP;
	
	timeout = 0xFFFFFF;
	while ((TWI0 -> TWI_SR & TWI_SR_TXCOMP) && timeout){
		timeout--;
	}
	if (!timeout) return 0;
	
	return TWI0 -> TWI_RHR;
}

uint8_t AK4588EN_Init(void)
{
	uint32_t timeout;
	
	TWI0 -> TWI_MMR = TWI_MMR_DADR(AK4588EN_I2C_ADDR);	//enter adress
	
	TWI0 -> TWI_THR = AK4588EN_PWRMNGMNT;		//register to start writing
	timeout = 0xFFFFFF;
	while ((TWI0 -> TWI_SR & TWI_SR_NACK) && timeout){
		timeout--;
	}
	if (!timeout) return 0;
	
	TWI0 -> TWI_THR = AK4588EN_PLL_CTRL_DEFAULT; //data	
	timeout = 0xFFFFFF;
	while ((TWI0 -> TWI_SR & TWI_SR_NACK) && timeout){
		timeout--;
	}
	if (!timeout) return 0;
	
	TWI0 -> TWI_THR = AK4588EN_DAC_TDM_DEFAULT; //data
	timeout = 0xFFFFFF;
	while ((TWI0 -> TWI_SR & TWI_SR_NACK) && timeout){
		timeout--;
	}
	if (!timeout) return 0;
	
	TWI0 -> TWI_THR = AK4588EN_CTRL1_DEFAULT; //data
	timeout = 0xFFFFFF;
	while ((TWI0 -> TWI_SR & TWI_SR_NACK) && timeout){
		timeout--;
	}
	if (!timeout) return 0;
	
	TWI0 -> TWI_THR = AK4588EN_CTRL2_DEFAULT; //data
	timeout = 0xFFFFFF;
	while ((TWI0 -> TWI_SR & TWI_SR_NACK) && timeout){
		timeout--;
	}
	if (!timeout) return 0;
	
	TWI0 -> TWI_THR = AK4588EN_MODE_CTRL_DEFAULT; //data
	timeout = 0xFFFFFF;
	while ((TWI0 -> TWI_SR & TWI_SR_NACK) && timeout){
		timeout--;
	}
	if (!timeout) return 0;
	
	TWI0 -> TWI_THR = AK4588EN_FILTER_SET_DEFAULT;	//data
	timeout = 0xFFFFFF;
	while ((TWI0 -> TWI_SR & TWI_SR_NACK) && timeout){
		timeout--;
	}
	if (!timeout) return 0;
	
	TWI0 -> TWI_THR = AK4588EN_HPF_EN_FILTER_SET_DEFAULT; //data
	timeout = 0xFFFFFF;
	while ((TWI0 -> TWI_SR & TWI_SR_NACK) && timeout){
		timeout--;
	}
	if (!timeout) return 0;
	
	TWI0 -> TWI_THR = AK4588EN_LOUT_VOLUME_DEFAULT; //data
	timeout = 0xFFFFFF;
	while ((TWI0 -> TWI_SR & TWI_SR_NACK) && timeout){
		timeout--;
	}
	if (!timeout) return 0;
	
	TWI0 -> TWI_THR = AK4588EN_ROUT_VOLUME_DEFAULT; //data
	timeout = 0xFFFFFF;
	while ((TWI0 -> TWI_SR & TWI_SR_NACK) && timeout){
		timeout--;
	}
	if (!timeout) return 0;
	

	
	//while (!(TWI0 -> TWI_SR & TWI_SR_TXRDY));
	
	TWI0 -> TWI_CR	= TWI_CR_STOP; //issue stop
	while (!(TWI0 -> TWI_SR & TWI_SR_TXCOMP)); //transfer not done yet
	
	//UART_Puts("Audio Codec Initialized\r\n");
	
	return 1;
}