import sys
import time

import numpy as np
from scipy import signal

# for soapy
sys.path.append('/usr/local/lib/python3.5/site-packages')
import SoapySDR
from SoapySDR import *
from numpy import abs
from numpy import sqrt, sum
from numpy.random import randn


def mixer(signal, lo_freq, fs):
    phase_inc = 2 * np.pi * lo_freq / fs
    phase_list = np.array(range(len(signal))) * phase_inc
    lo = np.exp(phase_list * 1j)

    mixed = signal * lo
    return mixed


class BladeDriver:
    def __init__(self, fs=2e6, bandwidth=2e6, frequency=382.5e6):
        self.fs = fs
        sdr = SoapySDR.Device()

        sdr.setSampleRate(SOAPY_SDR_RX, 0, fs)
        sdr.setSampleRate(SOAPY_SDR_TX, 0, fs)

        # NOTE: must have atleast 1 megas separation or PLLs start fighting
        sdr.setFrequency(SOAPY_SDR_RX, 0, frequency)
        sdr.setFrequency(SOAPY_SDR_TX, 0, frequency + 1.5e6)

        sdr.setBandwidth(SOAPY_SDR_RX, 0, bandwidth)
        sdr.setBandwidth(SOAPY_SDR_TX, 0, bandwidth)

        sdr.setGain(SOAPY_SDR_TX, 0, "VGA1", -35)

        sdr.setGain(SOAPY_SDR_RX, 0, "LNA", 6)
        sdr.setGain(SOAPY_SDR_RX, 0, "VGA1", 30)
        sdr.setGain(SOAPY_SDR_RX, 0, "VGA2", 30)
        self.sdr = sdr
        self.rxStream = sdr.setupStream(SOAPY_SDR_RX, "CF32", [0])

        # let things settle
        time.sleep(1)
        print('freq:', sdr.getFrequency(SOAPY_SDR_RX, 0))
        print('fs:', sdr.getSampleRate(SOAPY_SDR_RX, 0))
        print('bw:', sdr.getBandwidth(SOAPY_SDR_RX, 0))
        print('RX_LNA:', sdr.getGain(SOAPY_SDR_RX, 0, "LNA"))
        print('RX_VGA1:', sdr.getGain(SOAPY_SDR_RX, 0, "VGA1"))
        print('RX_VGA2:', sdr.getGain(SOAPY_SDR_RX, 0, "VGA2"))

    def get_total_rx_gain(self):
        return self.sdr.getGain(SOAPY_SDR_RX, 0, "LNA") + self.sdr.getGain(SOAPY_SDR_RX, 0, "VGA1") + \
               self.sdr.getGain(SOAPY_SDR_RX, 0, "VGA2") - 5

    def set_frequency(self, frequency):
        self.sdr.setFrequency(SOAPY_SDR_RX, 0, frequency)
        self.sdr.setFrequency(SOAPY_SDR_TX, 0, frequency + 1.5e6)
        time.sleep(0.1)

    def get_samples(self, seconds):
        chunk_size = 4096
        chunks_rx = int(seconds / (1 / self.fs) / chunk_size)
        self.sdr.activateStream(self.rxStream)

        rxBuff = np.array([[0] * chunk_size] * chunks_rx, np.complex64)

        for i in range(chunks_rx):
            while True:
                sr = self.sdr.readStream(self.rxStream, [rxBuff[i]], chunk_size)
                # print('ret:', sr.ret)  # num samples or error code
                # print('flags:', sr.flags)  # flags set by receive operation
                # print('time:', sr.timeNs)  # timestamp for receive buffer
                if sr.ret == chunk_size:
                    break

        self.sdr.deactivateStream(self.rxStream)
        rxBuff = np.hstack(rxBuff)
        rxBuff -= np.mean(rxBuff)
        return rxBuff

        # def __del__(self):
        #     self.sdr.closeStream(self.rxStream)


def iq_to_rssi(x, error=-5.1, gain_compensate=0.0, decimate=32):
    """

    :param error: default has been measured for BladeRF, full gains, 2M band 2.4G
    :param gain_compensate: decreases the RSSI for applied gain in the system
    :param decimate:  how much to decimate the result
    """

    dbm = 10 * np.log10(abs(x) ** 2 / 50) + 30  # 50 is ohms

    dbm -= gain_compensate  # compensate for GAINS
    dbm += error  # compensate error
    dbm = signal.resample_poly(dbm, 1, decimate)
    return dbm


# this is stolen from some python project
def awgn(input_signal, snr_dB, rate=1.0):
    """
    Addditive White Gaussian Noise (AWGN) Channel.
    Parameters
    ----------
    input_signal : 1D ndarray of floats
        Input signal to the channel.
    snr_dB : float
        Output SNR required in dB.
    rate : float
        Rate of the a FEC code used if any, otherwise 1.
    Returns
    -------
    output_signal : 1D ndarray of floats
        Output signal from the channel with the specified SNR.
    """

    avg_energy = sum(abs(input_signal) * abs(input_signal)) / len(input_signal)
    snr_linear = 10 ** (snr_dB / 10.0)
    noise_variance = avg_energy / (2 * rate * snr_linear)

    noise = (sqrt(noise_variance) * randn(len(input_signal))) + (sqrt(noise_variance) * randn(len(input_signal)) * 1j)

    output_signal = input_signal + noise

    return output_signal
