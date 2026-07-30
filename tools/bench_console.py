"""Interactive ST_Car balancing-bench console over the ST-Link virtual COM port."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import sys
import threading

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit(
        "pyserial is required. Install it with: python -m pip install pyserial"
    ) from exc


def choose_port(requested: str | None) -> str:
    if requested:
        return requested
    ports = list(list_ports.comports())
    if not ports:
        raise SystemExit("No serial ports found. Connect the NUCLEO ST-Link USB first.")
    for index, port in enumerate(ports, start=1):
        print(f"{index}: {port.device}  {port.description}")
    while True:
        choice = input("Select ST-Link VCP port number: ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            return ports[int(choice) - 1].device


def write_log(log_file, log_lock: threading.Lock, text: str) -> None:
    if log_file is None:
        return
    with log_lock:
        log_file.write(f"{datetime.now().isoformat(timespec='milliseconds')} {text}\n")
        log_file.flush()


def reader(port: serial.Serial, stopped: threading.Event, log_file,
           log_lock: threading.Lock) -> None:
    while not stopped.is_set():
        try:
            data = port.readline()
        except serial.SerialException as exc:
            print(f"\nSerial read failed: {exc}", file=sys.stderr)
            stopped.set()
            return
        if data:
            line = data.decode("utf-8", errors="replace").rstrip("\r\n")
            print(line)
            write_log(log_file, log_lock, f"< {line}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="COM port, for example COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--log", help="optional UTF-8 session log file")
    args = parser.parse_args()
    port_name = choose_port(args.port)
    stopped = threading.Event()
    log_lock = threading.Lock()
    log_file = None

    if args.log:
        log_path = Path(args.log)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_file = log_path.open("a", encoding="utf-8")
        print(f"Logging commands and replies to {log_path}")

    try:
        with serial.Serial(port_name, args.baud, timeout=0.1) as port:
            port.reset_input_buffer()
            print(f"Connected to {port_name} at {args.baud} 8N1.")
            print("This is the only serial terminal you need; do not open the same COM port elsewhere.")
            print("Type 'help'. Telemetry is quiet by default; use 'status' or 'stream on 200'.")
            print("Ctrl+C or Ctrl+Z then Enter exits. PC13 is bench emergency stop.")
            thread = threading.Thread(
                target=reader, args=(port, stopped, log_file, log_lock), daemon=True
            )
            thread.start()
            try:
                while not stopped.is_set():
                    line = input("bench> ")
                    write_log(log_file, log_lock, f"> {line}")
                    port.write((line + "\n").encode("ascii"))
                    port.flush()
            except (EOFError, KeyboardInterrupt):
                pass
            finally:
                stopped.set()
                try:
                    port.write(b"stop\nbench off\n")
                    port.flush()
                except serial.SerialException:
                    pass
                thread.join(timeout=0.5)
    finally:
        if log_file is not None:
            log_file.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
