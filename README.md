This is an implementation of a semiparametric EQ on SAMG55J19 Cortex-M4 ARM chip. The audio is received from an AK4558EN high fidelity audio chip via I2S. 

The goal of this project was to make a digital implementation of a semiparametric EQ, which meets a certain set of specifications, and with which the user interfaces in an analog manner (trim potentiometers read by an XMega microcontroller, which in turn communicates with the SAMG55 via UART). 
The specs were the following:

Specification | Value 
--- | --- 
Amplification range | -12 dB to +12 dB on center frequency
