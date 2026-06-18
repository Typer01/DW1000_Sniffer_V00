#!/usr/bin/env python3
"""
uwb_live.py — Real-time ESP-IDF log to pcap converter for DW1000 sniffer

Usage (pipe from idf.py monitor):
    idf.py monitor | python3 uwb_live.py

Usage (pipe from a serial port directly, e.g. with pyserial):
    python3 -m serial.tools.miniterm /dev/cu.usbserial-XXXX 115200 | python3 uwb_live.py

Usage (replay from a saved log file):
    cat saved_log.txt | python3 uwb_live.py

Output:
    uwb_capture.pcap  — appended to in real time, open in Wireshark
    uwb_capture.txt   — clean log with backtraces stripped

Wireshark tip:
    Open uwb_capture.pcap BEFORE running this script, then in Wireshark:
    Edit -> Preferences -> Capture -> check "Update list of packets in real time"
    Wireshark will show packets as they arrive.
"""

import re
import sys
import struct
import time
import os

# -----------------------------------------------------------------------
# Config
# -----------------------------------------------------------------------
PCAP_OUT  = "uwb_capture.pcap"
LOG_OUT   = "uwb_capture.txt"

# DLT_IEEE802_15_4 with FCS
PCAP_NETWORK = 195

# -----------------------------------------------------------------------
# Regex patterns
# -----------------------------------------------------------------------
# Matches a GOOD FRAME line — handles the SYS_STATUS running into hex bytes
FRAME_RE = re.compile(
    r'[IW] \((\d+)\) MAIN: GOOD FRAME RECEIVED, LEN=(\d+), SYS_STATUS=0x[0-9a-fA-F]{8}'
    r'([0-9a-fA-F]{2}(?: [0-9a-fA-F]{2})*)'
)

# Lines to suppress from clean log output
BACKTRACE_RE = re.compile(
    r'(Backtrace:|--- 0x|task_wdt:.*triggered|task_wdt:.*IDLE|task_wdt:.*running|task_wdt:.*CPU|task_wdt:.*Print)'
)

# -----------------------------------------------------------------------
# PCAP helpers
# -----------------------------------------------------------------------
def pcap_global_header():
    return struct.pack('<IHHiIII',
        0xa1b2c3d4,  # magic
        2, 4,         # version
        0,            # thiszone
        0,            # sigfigs
        65535,        # snaplen
        PCAP_NETWORK)

def pcap_record(ts_ms, raw_bytes):
    ts_sec  = ts_ms // 1000
    ts_usec = (ts_ms % 1000) * 1000
    n = len(raw_bytes)
    return struct.pack('<IIII', ts_sec, ts_usec, n, n) + raw_bytes

# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------
def main():
    packet_count = 0
    in_backtrace = False

    # Open pcap — write global header fresh each run
    pcap_file = open(PCAP_OUT, 'wb')
    pcap_file.write(pcap_global_header())
    pcap_file.flush()

    # Open clean log
    log_file = open(LOG_OUT, 'w')

    print(f"[uwb_live] Writing pcap -> {PCAP_OUT}", file=sys.stderr)
    print(f"[uwb_live] Writing log  -> {LOG_OUT}", file=sys.stderr)
    print(f"[uwb_live] Waiting for frames...", file=sys.stderr)

    try:
        for raw_line in sys.stdin:
            line = raw_line.rstrip()

            # ---- Backtrace detection ----
            if 'Task watchdog got triggered' in line:
                in_backtrace = True

            if in_backtrace:
                if BACKTRACE_RE.search(line):
                    continue  # suppress
                # Check if we're back to a normal MAIN log line
                if re.match(r'[IWED] \(\d+\) MAIN:', line):
                    in_backtrace = False
                else:
                    continue  # still in backtrace noise

            # ---- Write to clean log ----
            log_file.write(line + '\n')
            log_file.flush()

            # ---- Check for a good frame ----
            m = FRAME_RE.search(line)
            if m:
                ts_ms    = int(m.group(1))
                declared = int(m.group(2))
                hex_str  = m.group(3).strip()
                raw      = bytes(int(b, 16) for b in hex_str.split())

                if len(raw) != declared:
                    print(f"[uwb_live] WARNING: declared len={declared} but got {len(raw)} bytes", file=sys.stderr)

                # Write pcap record
                pcap_file.write(pcap_record(ts_ms, raw))
                pcap_file.flush()

                packet_count += 1
                print(
                    f"[uwb_live] PKT #{packet_count:04d}  ts={ts_ms}ms  "
                    f"len={len(raw)}  {raw[:4].hex()}...",
                    file=sys.stderr
                )

    except KeyboardInterrupt:
        print(f"\n[uwb_live] Stopped. {packet_count} packets written.", file=sys.stderr)
    finally:
        pcap_file.close()
        log_file.close()

if __name__ == '__main__':
    main()
