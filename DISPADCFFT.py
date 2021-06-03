# -*- coding: utf-8 -*-
"""
Created on Wed Jun  2 00:06:32 2021

         DISPADCFFT.py
         
         The purpose of this program is to collect data from the serial
         interface, containing frames of ADC values, and to transform these
         frames to Fourier space for analysis
         
         suggestion: in ipython console, type
         %matplotlib auto
         before running program

@author: papamidas DM1CR

credits to Joshua Hrisko
https://makersportal.com/blog/2018/8/14/real-time-graphing-in-python
"""

import serial
import keyboard
import time
import base64
import numpy as np
import matplotlib.pyplot as plt

plt.style.use('ggplot')

def live_plotter(x_vec,y1_data,line1,identifier='',pause_time=0.1):
    if line1==[]:
        # this is the call to matplotlib that allows dynamic plotting
        plt.ion()
        fig = plt.figure(figsize=(13,6))
        ax = fig.add_subplot(111)
        # create a variable for the line so we can later update it
        line1, = ax.plot(x_vec,y1_data,'-o',alpha=0.8)        
        #update plot label/title
        plt.ylim([0,150])
        plt.ylabel('FFT mag [dB]')
        plt.xlabel('FFT Frequency [Hz]')
        plt.show()
    
    plt.title('Frame {}'.format(identifier))
    # after the figure, axis, and line are created,
    # we only need to update the y-data
    line1.set_ydata(y1_data)
    # this pauses the data so the figure/axis can catch up -
    # the amount of pause can be altered by argument
    plt.pause(pause_time)
    
    # return line so we can update it again in the next iteration
    return line1


ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM12'
ser.timeout = 1.0

ser.open()
framecount = 0
correctframelength = 2048 # number of values in frame (not # of chars)
fs = 2.0e6 # this is the sample rate of the ADC
oldmag = np.zeros(int(correctframelength/2)+1)
line1 = []

# A kaiser window with beta=38 is good for really huge SFDR values
# see https://dm1cr.de/sdr-spurious-free-dynamic-range-sfdr-teil-1
# and is not really needed here, so you can choose other windows, too: 
fftwindow = np.kaiser(correctframelength,beta=38)
#fftwindow = np.hanning(correctframelength)
        
a = 0.1 # for FFT averaging
run = True

while run == True:
    try:
        encoded_frame = ser.readline()
        try:
            decoded_frame = base64.b64decode(encoded_frame[:-1])
            adc_frame = np.frombuffer(decoded_frame, dtype=np.uint16)
            framecount+=1
            frame_len = len(adc_frame)
            minADCVal = np.amin(adc_frame)
            maxADCVal = np.amax(adc_frame)
            noiseFloorCalcLen = int(frame_len/2 * 0.9)  # len for noise floor 
            print("adc frame_len = ", frame_len)
            print(framecount, frame_len, "")
            
            if (frame_len == correctframelength):
                meanvalue = np.mean(adc_frame)
                #print("mean = ", meanvalue)
                windowed_frame = fftwindow * (adc_frame - meanvalue)
                Y = np.fft.rfft(windowed_frame)
                freqs = np.fft.rfftfreq(frame_len, d=1.0/fs)# + 2*fs
                mag = np.abs(Y)
                newmag = a*mag + (1-a)*oldmag
                oldmag = np.copy(newmag)

                YdB = 20 * np.log10(newmag)
                iMax = np.argmax(YdB)
                mean = np.mean(newmag)
                sortedDB = np.sort(YdB)
                noisemean = np.mean(np.sort(YdB)[0:noiseFloorCalcLen])
                print(f'min ADC val = {minADCVal:},' +
                      f'max ADC Val = {maxADCVal:}')
                line1 = live_plotter(freqs,YdB,line1,
                  f'#{framecount}, max: {YdB[iMax]:.1f} dB at index {iMax}' +
                  f' / {freqs[iMax]:.1f} Hz' +
                  f', noise floor at {noisemean:.1f} dB')
            else:
                print("err, frame_len = ", frame_len)
        except Exception as inst:
            print("--exception: ")
            print(type(inst))
            print(inst.args)
            print(inst)

        if keyboard.is_pressed("x"):
           oldmag = np.zeros(int(correctframelength/2)+1)
           framecount = 0

    except serial.SerialException:
        print("Serial Exception")
        time.sleep(1)
        
    except KeyboardInterrupt:
        print("keyboard interrupt, closing port ", ser.port)
        ser.close()
        run = False
     
        