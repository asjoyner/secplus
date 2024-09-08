#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Secplus Rx
# GNU Radio version: 3.10.9.2

from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5 import QtCore
from PyQt5.QtCore import QObject, pyqtSlot
from gnuradio import blocks
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
import math
import osmosdr
import time
import secplus_rx_secplus_decode as secplus_decode  # embedded python block
import secplus_rx_secplus_v2_decode as secplus_v2_decode  # embedded python block
import sip
import socket
import sys
import datetime
import hashlib

class GarageDoor():
    """This class responds to Security+ events.

    It's primary function is to make authorization decisions based on valid
    codes, time-of-day, and other factors.  If appropriate it then triggers a
    GPIO pin to open a garage door.
    """

    def __init__(self):
        """Setup some constants and prepare to handle events."""
        self.hostname = socket.gethostname()
        print(f"Hostname is: {self.hostname}")

        # I chose to use this command to store a representation of the fixed
        # code securely in the code:
        # $ echo -n "$(hostname)<fixedcode>" | sha256sum | cut -f 1 -d " "  
        # This computes a sha256 hash of the fixed code, "salted" with the
        # hostname of the machine.  This avoids the need for a config file, and
        # prevents someone clever from just computing the sha256 of all the
        # integers as strings to derive my fixed garage door code.
        self.fixed_code_hash = "51efcfbe00f5508736fe78b0802d08ace9531b22aaa9e71f97bcd181f4d83c61"

        # If you're not going to publish this file on github, you can use this
        # simpler method, instead:
        self.fixed_code = ""

        # This implementation accepts the first rolling code it sees from the
        # fixed code above, stores the rolling code, and accepts any future
        # codes in these ranges:
        self.rolling_code_min = 10   # number of codes in the "past"
        self.rolling_code_max = 1000 # number of missed codes in the "future"

        # The reduces the writes to the file on the SDCard, but slighty
        # increases the risk of losing sync.
        self.rolling_code_update_interval = 250

        self.rolling_code = 0
        self.rcpath = "/tmp/rolling_code.txt"
        try:
            with open(self.rcpath, "r+") as rcfile:
                self.rolling_code = int(rcfile.read().strip())

        except IOError as err:
            print(f"Could not open rolling code file ({self.rcpath}): {err}")
            print("Accepting and storing the first rolling code that is decoded.")
        except ValueError as err:
            print(f"Could not parse rolling code file ({self.rcpath}): {err}")
            print("Accepting and storing the first rolling code that is decoded.")

    def event(self, rolling, fixed):
        rolling = int(rolling)
        print(f"Receivd event: {rolling}, {fixed}")
        if self.fixed_code:
            if self.fixed_code != fixed:
                print("Fixed code did not match.")
                return
        else:
            hv = hashlib.sha256()
            hv.update(self.hostname.encode())
            hv.update(f"{fixed}".encode())
            if self.fixed_code_hash != hv.hexdigest():
                print("Fixed code did not match the hash: ", hv.hexdigest())
                return
            self.fixed_code = fixed  # save some hash steps in the future

        print("Fixed code matched")

        if self.rolling_code != 0:  # skip the range checks if we don't know the code yet
            if rolling < (self.rolling_code - self.rolling_code_min):
                print(f"Rolling code too far behind ({rolling} < {self.rolling_code})")
                return
            if rolling > (self.rolling_code + self.rolling_code_max):
                print(f"Rolling code too far ahead ({rolling} > {self.rolling_code})")
                return
        print("Rolling code matched")
        if rolling > self.rolling_code + self.rolling_code_update_interval:
            try:
                with open(self.rcpath, "w+") as rcfile:
                    rcfile.write(f"{rolling}")
                    print("Rolling code saved to disk.")

            except IOError as err:
                print(f"Could not update rolling code file ({self.rcpath}): {err}:")
        self.rolling_code = rolling
        self.open()
            

    def open(self):
        """This is triggered when the door should be opened."""
        print("Opening the door")

    def serve(self):
        """TODO: Add HTTP server for status, and overrides."""
        while True:
            time.sleep(60)


class secplus_rx(gr.top_block):
    """Configures the SDR device to read Security+ codes and send callbacks."""

    def __init__(self, callback):
        gr.top_block.__init__(self, "Secplus Rx", catch_exceptions=True)

        ##################################################
        # Variables
        ##################################################
        self.threshold = threshold = 0.10
        self.samp_rate = samp_rate = 2000000
        self.freq = freq = 390150000
        self.decim2 = decim2 = 50
        self.decim1 = decim1 = 2

        ##################################################
        # Blocks
        ##################################################
        self.secplus_v2_decode = secplus_v2_decode.blk(samp_rate=samp_rate // decim1 // decim2, threshold=threshold, callback=callback)
        self.secplus_decode = secplus_decode.blk(samp_rate=samp_rate // decim1 // decim2, threshold=threshold, callback=callback)
        self.rational_resampler_xxx_1 = filter.rational_resampler_fff(
                interpolation=1,
                decimation=decim2,
                taps=[1.0/decim2]*decim2,
                fractional_bw=0)
        self.rational_resampler_xxx_0 = filter.rational_resampler_ccc(
                interpolation=1,
                decimation=decim1,
                taps=[],
                fractional_bw=0)

        self.osmosdr_source_0 = osmosdr.source(args="numchan=1")
        self.osmosdr_source_0.set_time_unknown_pps(osmosdr.time_spec_t())
        self.osmosdr_source_0.set_sample_rate(samp_rate)
        self.osmosdr_source_0.set_center_freq((freq - 300e3), 0)
        self.osmosdr_source_0.set_freq_corr(0, 0)
        self.osmosdr_source_0.set_dc_offset_mode(0, 0)
        self.osmosdr_source_0.set_iq_balance_mode(0, 0)
        self.osmosdr_source_0.set_gain_mode(False, 0)
        self.osmosdr_source_0.set_gain(30, 0)
        self.osmosdr_source_0.set_if_gain(32, 0)
        self.osmosdr_source_0.set_bb_gain(32, 0)
        self.osmosdr_source_0.set_antenna('', 0)
        self.osmosdr_source_0.set_bandwidth(1e6, 0)

        self.blocks_rotator_cc_0 = blocks.rotator_cc((2 * math.pi * -300e3 / samp_rate), False)
        self.blocks_complex_to_mag_0 = blocks.complex_to_mag(1)

        ##################################################
        # Connections
        ##################################################
        self.connect((self.osmosdr_source_0, 0), (self.blocks_rotator_cc_0, 0))
        self.connect((self.blocks_rotator_cc_0, 0), (self.rational_resampler_xxx_0, 0))
        self.connect((self.rational_resampler_xxx_0, 0), (self.blocks_complex_to_mag_0, 0))
        self.connect((self.blocks_complex_to_mag_0, 0), (self.rational_resampler_xxx_1, 0))
        self.connect((self.rational_resampler_xxx_1, 0), (self.secplus_decode, 0))
        self.connect((self.rational_resampler_xxx_1, 0), (self.secplus_v2_decode, 0))


def main():

    door = GarageDoor()
    tb = secplus_rx(door.event)

    tb.start()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()
        sys.exit(1)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    door.serve()

if __name__ == '__main__':
    main()
