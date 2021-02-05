/*
 * PIO.h
 *
 * Created: 22-Apr-18 17:16:21
 *  Author: Willem van der Kooij
 */ 


#ifndef PIO_H_
#define PIO_H_

void Init_PIO(void);

void Led3State(uint8_t state);
void AK4558EN_PDN_State(uint8_t state);

#endif /* PIO_H_ */