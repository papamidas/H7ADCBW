# -*- coding: utf-8 -*-
"""apertureJitter.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1HriKPzfSfQeJqMilkGbmS9wleWajFtys
"""

# Simulations with a simple model of the internal 16-bit-ADC of a STM32H743ZI
# that tries to explain the measured curves in the blog entry
# https://dm1cr.de/bandpass-sampling-and-analog-bandwidth-of-the-16-bit-adc-peripheral-of-the-stm32h7-part-ii
# 
# The model assumes an internal sampling capacitor (of value CADC)
# that is connected to the input terminals of the MCU via an internal
# switch (with a resistance of RADC) during a time interval called
# "samplingTime". The time instant where this interval begins and the
# time instant where this interval ends is somewhat uncertain.
# The amount of uncertainty is called "rmsJitter".
# After the switch opens, the voltage across the sampling capacitor
# gets measured by the ADC.
# The RC time contant, formed by the switch resistance and the 
# capacitance of the sampling capacitor, together with the uncertainty
# of the closing times and opening times of the switch, lead to a
# frequency response of the ADC values that correspond well to the
# measured data.
# More explanations can be found on dm1cr.de
# DM1CR, Mar 11, 2022
from numpy.random import default_rng
import numpy as np
import random

plot = 'No'
CADC = 4.0e-12
RADC = 2000
sampleCount = 2048
sampleRate = 2.0e6
samplingPeriod = 1/sampleRate
h7adcclock = sampleRate * 10
samplingTime = 1.5 * 1.0/h7adcclock
rmsJitter = 100.0e-12
sigFreq = 5.5e6
sigAmp = 25800

freqlist = [0.5,1.5,2.5,3.5,4.5,5.5,6.5,7.5,8.5,9.5,10.5,11.5,12.5,13.5,
            14.5,15.5,16.5,17.5,18.5,19.5,20.5,30.5,40.5,50.5,60.5,70.5,
            80.5,90.5,100.5,200.5,300.5,400.5,500.5,600.5,700.5,800.5,900.5,
            1000.5,1100.5,1200.5]

for f_MHz in freqlist:
  sigFreq = f_MHz * 1.0e6
  oldmag = np.zeros(int(sampleCount/2)+1)
  a = 0.1 # for FFT averaging
  fftwindow = np.kaiser(sampleCount,beta=38)

  for iter in range(int(10/a)):
    startPhase = 2.0 * np.pi * random.random()
    rng = default_rng()
    # generate time jitter values for the start of the sampling interval:
    jitterVals = rmsJitter * rng.standard_normal(sampleCount)
    # ...and another one for the end of the sampling interval:
    jitterVals2 = rmsJitter * rng.standard_normal(sampleCount)

    # generate an array of sample times with jitter:
    t = np.arange(0, sampleCount*samplingPeriod, samplingPeriod) + jitterVals
    # generate an array of jittered sampling intervals:
    samplingIntervals = samplingTime + jitterVals + jitterVals2
 
    # the following lines calculate the voltage across C_adc at the end of the sampling time:
    numCyclesInSamplingTime = samplingIntervals / sigPeriod
    # calculate the number of iteration steps for charging C_adc:
    sigPeriod = 1.0/sigFreq
    iterationsPerSigPeriod = 32
    numTSteps = int(samplingTime/sigPeriod * iterationsPerSigPeriod)
    if numTSteps < 20:
      numTSteps = 20
    stepDuration = samplingIntervals/numTSteps
    phaseStep = 2 * np.pi * numCyclesInSamplingTime / numTSteps
    # at first calculate array of sampled phases of the signal
    # and then calculate phases of the sampled signal voltages:
    phaseArr = 2.0 * np.pi * sigFreq * t + startPhase
    adc_frame = sigAmp * np.sin(phaseArr)
    # set voltage across C_adc to zero at beginning of each sampling interval:
    uCadc = np.zeros(sampleCount)
    for step in range(numTSteps):
      # now calculate the voltage across C_adc after one time step:
      # using delta U = Q/C = i*delta_t/C
      uCadc = (adc_frame-uCadc)/RADC * stepDuration / CADC + uCadc
      # calculate next values of signal voltages:
      phaseArr += phaseStep
      adc_frame = sigAmp * np.sin(phaseArr)
    # simulate common mode voltage offset and 16 bit digitization:
    adc_frame = np.uint16( 32767 + uCadc)
    # for the rest of the calculations, do exactly the same as for the
    # measured data:
    meanvalue = np.mean(adc_frame)
    windowed_frame = fftwindow * (adc_frame - meanvalue)
    Y = np.fft.rfft(windowed_frame)
    freqs = np.fft.rfftfreq(sampleCount, d=1.0/sampleRate)
    mag = np.abs(Y)
    newmag = a*mag + (1-a)*oldmag # simple averaging
    oldmag = np.copy(newmag)

  YdB = 20 * np.log10(newmag)
  iMax = np.argmax(YdB)
  mean = np.mean(newmag)
  minADCVal = np.amin(adc_frame)
  maxADCVal = np.amax(adc_frame)
  sortedDB = np.sort(YdB)
  noiseFloorCalcLen = int(sampleCount/2 * 0.9)  # len for noise floor 
  noisemean = np.mean(np.sort(YdB)[0:noiseFloorCalcLen])
  print(f'{sigFreq/1.0e6}\t' +
        f'{YdB[iMax]:.1f}\t' +
        f'{noisemean:.1f}\t' +
        f'{numTSteps:.1f}')
  

  if plot == 'yes':
    import matplotlib.pyplot as plt
    plt.style.use('ggplot')

    fig = plt.figure(figsize=(13,6))
    ax = fig.add_subplot(111)
    ax.plot(freqs,YdB,'-o',alpha=0.8)        
    plt.ylim([0,150])
    plt.ylabel('FFT mag [dB]')
    plt.xlabel('FFT Frequency [Hz]')
    identifier = f', max: {YdB[iMax]:.1f} dB at index {iMax} / {freqs[iMax]:.1f} Hz, noise floor at {noisemean:.1f} dB'
    plt.title('Simulation {}'.format(identifier))
    plt.show()