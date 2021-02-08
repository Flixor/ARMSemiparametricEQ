## Overview
This is an implementation of a semiparametric EQ on SAMG55J19 Cortex-M4 ARM chip. The audio is received from an AK4558EN high fidelity audio chip via I2S. 

The goal of this project was to make a digital implementation of a semiparametric EQ, which meets a certain set of specifications. The EQ is semiparametric in the sense that the user can adjust gain and center frequency, but the Q factor is constant. The user interfaces with the EQ in an analog manner, with trim potentiometers for gain and frequency, and a switch for boost/cut.

There is some unresolved discussion in the audio world regarding what 'constant Q' actually means in practice for parametric EQs (not including shelves and LPF/HPF). Most hardware EQs have a constant skirt, but not constant Q; meaning the frequency area that the EQ affects always remains the same, but the peak of the bell becomes sharper as more gain is applied. A perfect constant Q EQ would keep the shape of the bell the same regardless of gain, but the skirt would become wider. 
The latter variant is possible in the omnipotent digital realm, but a lot harder to replicate as an analog implementation. As a compromise, a design has been implemented where a bandpass output of a (frequency adjustable) state variable filter is controllably summed with the EQ input.

The 'codesign' naming comes from the overarching project this was a part of, which involved creating an analog and digital implementation of a device that meet the same specifications. The analog version was used to power the SAMG55 version, and also provided the user control: the trimpots are read by an XMega microcontroller, which in turn communicates with the SAMG55 via UART. The analog version will be uploaded soon to a separate repo. 

## Specifications
The specs are the following:

Specification | Value 
--- | --- 
Amplification range | -12 dB to +12 dB at center frequency
Amplification control | 0 - 6 V (0 - 12 dB)
Boost/cut symmetry | Boost/cut switching only causes amplification to flip, ie. to be mirrored in the 0 dB line
Boost/cut symmetry accuracy | Max 1% deviation
Frequency range | 250 Hz - 4 kHz
Frequency control | 250 mV - 4 V (250 Hz - 4 kHz)
Control accuracy | Max 1% deviation of maximum value
Q factor definition | -3 dB points from peak at positive gain, +3 dB points from dip at negative gain
Q factor at +/- 6 dB gain | 1.414 (3 dB points are 0.5 octave from center frequency)
Q factor at +/- 12 dB gain | 1.876 (3 dB points are 0.38 octave from center frequency)
Q factor accuracy | Max 1% deviation
THD+N | 1% of signal strength, rms, 20 Hz - 22 kHz
Input signal | Max 1 V peak (2 V peak-peak)
Flatness | <0.5 dB at 0 dB gain

## Hardware
The hardware design for the SAMG55 board and AK4558EN board are unfortunately not mine to share. If interested, shoot me a message!