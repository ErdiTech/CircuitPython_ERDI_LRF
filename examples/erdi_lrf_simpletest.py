# SPDX-FileCopyrightText: 2026 ERDI TECH LTD
# SPDX-License-Identifier: MIT
"""Read an ERDI LRF22VB (460800 baud stream) with busio.UART and print the distance."""
import time

import board
import busio

import erdi_lrf

uart = busio.UART(board.TX, board.RX, baudrate=460800, timeout=0)
lrf = erdi_lrf.ErdiLRF(uart, "LRF22VB")

while True:
    d = lrf.read_latest()
    print("out of range / no frame yet" if d is None else "%.3f m" % d)
    time.sleep(0.05)
