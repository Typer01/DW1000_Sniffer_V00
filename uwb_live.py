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

# -----------------------------------------------------------------------
# Config
# -----------------------------------------------------------------------
PCAP_OUT  = "uwb_capture.pcap"
LOG_OUT   = "uwb_capture.txt"

# DLT_IEEE802_15_4 with FCS
PCAP_NETWORK = 195

# Number of lines to wait, after a "GOOD FRAME RECEIVED" header, for the
# hex-dump line before giving up on that frame.
MAX_LINES_AWAITING_HEX = 8

# Safety valve: if a backtrace is never followed by a recognizable log line
# (e.g. serial noise during reset), stop suppressing after this many lines
# so the script can't get stuck silently swallowing output forever.
MAX_BACKTRACE_SUPPRESS_LINES = 500

# -----------------------------------------------------------------------
# Regex patterns
# -----------------------------------------------------------------------
# Strips ANSI color escape codes. idf_monitor.py colorizes I/W/E log lines
# by default (independent of the firmware's CONFIG_LOG_COLORS setting), so
# every line arrives wrapped in e.g. "\x1b[0;32m...\x1b[0m".
ANSI_RE = re.compile(r'\x1b\[[0-9;]*m')

# Matches the "GOOD FRAME RECEIVED" header line. The hex payload is NOT on
# this line — it's printed via a separate raw printf() call on its own
# line afterwards, with no log prefix at all.
FRAME_HEADER_RE = re.compile(
    r'\((\d+)\) MAIN: GOOD FRAME RECEIVED, LEN=(\d+), SYS_STATUS=0x[0-9a-fA-F]{8}'
)

# A line that is purely space-separated hex bytes (the raw printf dump).
HEX_LINE_RE = re.compile(r'^([0-9A-Fa-f]{2}(?: [0-9A-Fa-f]{2})*)$')

# Anything that starts a crash/backtrace dump worth suppressing.
BACKTRACE_START_RE = re.compile(
    r'Task watchdog got triggered|Guru Meditation Error|abort\(\) was called'
)

# A normal tagged log line, used to detect that backtrace/reboot noise has
# ended and regular logging has resumed (any tag, not just MAIN — after a
# crash reboot, the first lines back are usually "boot:", "cpu_start:", etc).
NORMAL_LOG_LINE_RE = re.compile(r'^[IWEV] \(\d+\) \S+:')

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
    backtrace_suppressed_lines = 0

    # Pending frame header, waiting for its hex-dump line to arrive.
    pending_ts_ms = None
    pending_declared_len = None
    pending_lines_waited = 0

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
            line = ANSI_RE.sub('', raw_line).rstrip()

            # ---- Backtrace detection ----
            if not in_backtrace and BACKTRACE_START_RE.search(line):
                # Suppress the trigger line itself unconditionally — it's
                # often a tagged line (e.g. "W (...) task_wdt: ...") that
                # would otherwise satisfy the recovery check below on this
                # same pass.
                in_backtrace = True
                backtrace_suppressed_lines = 1
                continue

            if in_backtrace:
                backtrace_suppressed_lines += 1
                if NORMAL_LOG_LINE_RE.match(line) or backtrace_suppressed_lines > MAX_BACKTRACE_SUPPRESS_LINES:
                    in_backtrace = False
                else:
                    continue  # suppress backtrace/reboot noise

            # ---- Write to clean log ----
            log_file.write(line + '\n')
            log_file.flush()

            # ---- Check for a new good-frame header ----
            m = FRAME_HEADER_RE.search(line)
            if m:
                pending_ts_ms = int(m.group(1))
                pending_declared_len = int(m.group(2))
                pending_lines_waited = 0
                continue

            # ---- Waiting for the hex-dump line that follows a header ----
            if pending_declared_len is not None:
                pending_lines_waited += 1

                hex_match = HEX_LINE_RE.match(line)
                if hex_match:
                    raw = bytes(int(b, 16) for b in hex_match.group(1).split())

                    if len(raw) != pending_declared_len:
                        print(f"[uwb_live] WARNING: declared len={pending_declared_len} but got {len(raw)} bytes", file=sys.stderr)

                    pcap_file.write(pcap_record(pending_ts_ms, raw))
                    pcap_file.flush()

                    packet_count += 1
                    print(
                        f"[uwb_live] PKT #{packet_count:04d}  ts={pending_ts_ms}ms  "
                        f"len={len(raw)}  {raw[:4].hex()}...",
                        file=sys.stderr
                    )

                    pending_ts_ms = None
                    pending_declared_len = None
                    pending_lines_waited = 0
                elif pending_lines_waited >= MAX_LINES_AWAITING_HEX:
                    # Frame was likely out of range (len==0 or > FRAME_LEN_MAX)
                    # and the firmware never printed a hex dump for it.
                    pending_ts_ms = None
                    pending_declared_len = None
                    pending_lines_waited = 0

    except KeyboardInterrupt:
        print(f"\n[uwb_live] Stopped. {packet_count} packets written.", file=sys.stderr)
    finally:
        pcap_file.close()
        log_file.close()

if __name__ == '__main__':
    main()
