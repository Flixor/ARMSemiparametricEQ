/*
 * I2SC.h
 *
 * Created: 19-Mar-18 13:50:54
 *  Author: Willem van der Kooij
 */ 


#ifndef I2SC_H_
#define I2SC_H_

void Init_I2SC(int32_t *DMAsrcbuffL, int32_t *DMAsrcbuffR, int32_t *DMAdstbuffL, int32_t *DMAdstbuffR, uint32_t DMAbuffLen);
void I2SC_DMAenable(int32_t *srcbuffL, int32_t *srcbuffR, int32_t *dstbuffL, int32_t *dstbuffR, uint32_t len);
void I2SC_WriteData(uint32_t txdata);
uint32_t I2SC_ReadData(void);

int32_t TwosComplementConvert(int32_t data);


#endif /* I2SC_H_ */