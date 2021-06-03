# H7ADCBW
Firmware and Python script for measuring the analog bandwidth of the ADC in a H7 for bandpass sampling applications

This is a simple firmware for doing test measurements on the built-in ADC1 of a STM32H743ZI microcontroller.
The firmware runs on a NUCLEOH743ZI-Board, built by ST Microelectronics and available from many vendors.
The firmware was built with STM32CubeIDE version 1.6.1, which can be downloaded from ST.com

The firmware captures ADC data from the differential inputs PF12 und PF11 and sends them out via the USART3 peripheral.
The Python script DISPADCFFT.py takes the data, performs a FFT and displays the results.
The Python script was tested with Spyder 3.3.6 and Python 3.7.4
In addition, the keyboard library was used, which can be installed from an Anaconda prompt via "pip install keyboard"
This keyboard library is used for resetting the averaged live data by the user through pressing the "x" key, e.g. after a change of input frequency.

In order to measure ADC input frequencies above a few Megahertz, you should think about signal integrity issues.
The NUCLEO-H743ZI is not actually built for feeding MHz-signals to the ADC inputs, so you have to think about techniques for doing it as well as possible.

Some results can be found on my [homepage](https://dm1cr.de/).

Some pictures, showing ways of how to connect RF signals to the ADC inputs, can be found there, too.


