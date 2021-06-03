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
Before using the Python script, change the COM-port of your ST-link-adapter or J-Link-Adapter to the right port number.
You can use the Windows device manager for finding out the correct port number.

In addition, choose the right matplotlib backend for displaying live data in a correct way.
"%matplotlib auto" sets the correct backend for my ipython console.

In order to measure ADC input frequencies above a few Megahertz and at the same time at very high signal-to-distortion ratios, you should always think about signal integrity issues.
The NUCLEO-H743ZI is not actually built for feeding MHz-signals to the ADC inputs or for measuring verly low voltage levels, so you have to think by yourself about techniques for doing it as well as possible.

Some pictures, showing ways of how to connect RF signals to the ADC inputs, can be found there, too.

Some results of the measurements can be found on my [homepage](https://dm1cr.de/).



