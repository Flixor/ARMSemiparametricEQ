This is an implementation of a semiparametric EQ on SAMG55J19 Cortex-M4 ARM chip. The audio is received from an AK4558EN high fidelity audio chip via I2S. 

The goal of this project was to make a digital implementation of a semiparametric EQ, which meets a certain set of specifications. The user interfaces with the EQ in an analog manner, using trim potentiometers read by an XMega microcontroller, which in turn communicates with the SAMG55 via UART. The EQ is semiparametric in the sense that the user can adjust gain and center frequency, but the Q factor is constant. 

The 'codesign' naming comes from the overarching project this was a part of, which involved creating an analog and digital implementation of a device that meet the same specifications.  

The specs are the following:

Specification | Value 
--- | --- 
Amplification range | -12 dB to +12 dB on center frequency
Amplification control | 0 - 6 V (0 - 12 dB)
Boost/cut symmetry | Boost/cut switching only causes amplification to flip, ie. to be mirrored in the 0 dB line. Max. 1% deviation.