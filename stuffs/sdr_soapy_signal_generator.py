import math
import time
from threading import Thread, Event

import numpy as np

import SoapySDR
from SoapySDR import *


class SigGen_thread(Thread):
    def __init__(self, sdr, ampl, waveFreq):
        Thread.__init__(self)
        self.sdr = sdr
        self.ampl = ampl
        self.rate = sdr.getSampleRate(SOAPY_SDR_TX, 0)

        self.txStream = sdr.setupStream(SOAPY_SDR_TX, "CF32", [0])
        self.phaseAcc = 0
        self.phaseInc = 2 * math.pi * waveFreq / self.rate
        self.streamMTU = sdr.getStreamMTU(self.txStream)
        self.sampsCh0 = np.array([ampl] * self.streamMTU, np.complex64)

        self.__stop = Event()

    def stop(self):
        self.__stop.set()

    def run(self):
        self.sdr.activateStream(self.txStream)
        while True:
            if self.__stop.is_set():
                self.sdr.deactivateStream(self.txStream)
                self.sdr.closeStream(self.txStream)
                return

            phaseAccNext = self.phaseAcc + self.streamMTU * self.phaseInc
            self.sampsCh0 = self.ampl * np.exp(1j * np.linspace(self.phaseAcc, phaseAccNext, self.streamMTU)).astype(
                np.complex64)
            self.phaseAcc = phaseAccNext
            while self.phaseAcc > math.pi * 2:
                self.phaseAcc -= math.pi * 2

            sr = self.sdr.writeStream(self.txStream, [self.sampsCh0], self.sampsCh0.size)
            if sr.ret != self.sampsCh0.size:
                raise Exception("Expected writeStream() to consume all samples! %d" % sr.ret)


class SignalGenerator:
    """
    Starts signal generator in new thead ::

        sdr = SignalGenerator(frequency=2405.35e6, fs=1e6, bandwidth=1e6)
        sdr.start_sig_generator()

    """

    def __init__(self, fs=2e6, bandwidth=2e6, frequency=382.5e6):
        sdr = SoapySDR.Device()

        sdr.setSampleRate(SOAPY_SDR_RX, 0, fs)
        sdr.setSampleRate(SOAPY_SDR_TX, 0, fs)

        # HITLER: must have atleast 1 megas separation or PLLs start fighting
        sdr.setFrequency(SOAPY_SDR_RX, 0, frequency + 1.5e6)
        sdr.setFrequency(SOAPY_SDR_TX, 0, frequency)

        sdr.setBandwidth(SOAPY_SDR_RX, 0, bandwidth)
        sdr.setBandwidth(SOAPY_SDR_TX, 0, bandwidth)

        sdr.setGain(SOAPY_SDR_TX, 0, "VGA1", -35)

        sdr.setGain(SOAPY_SDR_RX, 0, "LNA", 0)
        sdr.setGain(SOAPY_SDR_RX, 0, "VGA1", 5)
        sdr.setGain(SOAPY_SDR_RX, 0, "VGA2", 0)
        self.sdr = sdr
        self.sig_gen = SigGen_thread(sdr, 1.0, -800e3)

    def start_sig_generator(self):
        self.sig_gen.start()
        time.sleep(0.2)

    def set_frequency(self, frequency):
        # todo: this may be fucked up
        self.sdr.setFrequency(SOAPY_SDR_RX, 0, frequency + 1.5e6)  # to avid PLL fight
        self.sdr.setFrequency(SOAPY_SDR_TX, 0, frequency)

    def __del__(self):
        self.sig_gen.stop()
        self.sig_gen.join()

    def set_output_gain(self, param):
        self.sdr.setGain(SOAPY_SDR_TX, 0, "VGA1", param)


if __name__ == '__main__':
    sdr = SignalGenerator(frequency=2405.35e6, fs=1e6, bandwidth=1e6)
    sdr.set_output_gain(-5)
    sdr.start_sig_generator()
