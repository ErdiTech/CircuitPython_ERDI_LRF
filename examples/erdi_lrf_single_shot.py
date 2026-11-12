# SPDX-FileCopyrightText: 2026 ERDI TECH LTD
# SPDX-License-Identifier: MIT
"""Poll an ERDI LRF3000A1 (115200 baud, single-shot protocol) with busio.UART."""
import time

import board
import busio

import erdi_lrf

uart = busio.UART(board.TX, board.RX, baudrate=115200, timeout=0)
lrf = erdi_lrf.ErdiLRF(uart, "LRF3000A1")

while True:
    try:
        d = lrf.measure(timeout_ms=1000)
        print("no valid echo" if d is None else "%.1f m" % d)
    except OSError as err:
        print(err)
    time.sleep(1)
