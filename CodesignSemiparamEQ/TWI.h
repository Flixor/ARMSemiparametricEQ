/*
 * TWI.h
 *
 * Created: 19-Mar-18 17:04:25
 *  Author: Willem van der Kooij
 */ 


#ifndef TWI_H_
#define TWI_H_

void Init_TWI(void);
void TWI_WriteByte(uint8_t slaveAddr, uint8_t txdata);
uint8_t TWI_Read(uint8_t slaveAddr, uint8_t RegAddr);


#endif /* TWI_H_ */