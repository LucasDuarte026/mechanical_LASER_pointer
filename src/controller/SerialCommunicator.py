"""Thread-safe serial terminal helper for the Arduino-based laser rig."""

from __future__ import annotations

import threading
import time
from typing import Optional

import serial
import serial.tools.list_ports


class ArduinoSerialTerminal:
    """Manage scanning, connecting and exchanging data with an Arduino board.

    The class owns the serial handle and a background reader thread so that
    asynchronous logs emitted by the firmware do not clash with the interactive
    prompt. Calls to :meth:`send` are guarded by ``self.write_lock`` so UI
    actions, the controller loop and the demo script can all share the same
    connection safely.
    """

    def __init__(self, baud_rate: int = 9600) -> None:
        self.baud_rate = baud_rate
        self.port: Optional[str] = None
        self.ser: Optional[serial.Serial] = None
        self.running = False
        self._reader_thread: Optional[threading.Thread] = None
        self._writer_thread: Optional[threading.Thread] = None
        self.write_lock = threading.Lock()

    @staticmethod
    def scan_for_arduino() -> Optional[str]:
        """Return the first serial port that looks like an Arduino."""
        print("Looking for an Arduino-compatible serial device...")
        arduino_keywords = ["arduino", "ch340", "usb-serial", "cp210x", "ftdi"]
        ports = serial.tools.list_ports.comports()

        for port in ports:
            desc = str(port.description or "").lower()
            mf = str(port.manufacturer or "").lower()
            hwid = str(port.hwid or "").lower()
            for keyword in arduino_keywords:
                if keyword in desc or keyword in mf or keyword in hwid:
                    print("\n[Success] Arduino detected!")
                    print(f"  Port: {port.device}")
                    print(f"  Description: {port.description}")
                    return port.device

        print("\n[Warning] No Arduino device found.")
        return None

    def _read_from_port(self) -> None:
        """Continuously stream incoming lines from the board."""
        while self.running and self.ser:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        # Carriage return keeps the prompt on the same line.
                        print(f"\rArduino: {line}\nYou: ", end="")
            except (serial.SerialException, OSError):
                if self.running:
                    print("\n[Error] Serial connection lost. Reader thread stopping.")
                break

    def _write_loop(self) -> None:
        """Interactive REPL so humans can send raw commands."""
        print("\n--- Serial Terminal Started ---")
        print("Type 'exit' to quit the terminal or enter raw commands.")
        print("--------------------------------")
        print("You: ", end="")

        while self.running:
            try:
                msg = input()

                if not self.running:
                    break

                if msg.lower() == "exit":
                    print("Leaving the serial terminal...")
                    self.stop_terminal()
                    break

                if msg.strip() == "c":
                    self.send("50 50")
                else:
                    self.send(msg)

            except (KeyboardInterrupt, EOFError):
                print("\n[Info] Terminal interrupted by user.")
                self.stop_terminal()
                break
            except Exception as exc:
                if self.running:
                    print(f"\n[Error] Input loop failed: {exc}")
                self.stop_terminal()
                break

        print("[Info] Writer thread stopped.")

    def start_terminal(self, port_name: str) -> bool:
        """Open the serial device and start reader/writer threads."""
        self.port = port_name
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.running = True

            print(f"\nConnected to {self.port} at {self.baud_rate} baud.")
            print("Giving the Arduino a couple of seconds to boot...")
            time.sleep(2)

            self._reader_thread = threading.Thread(target=self._read_from_port, daemon=True)
            self._reader_thread.start()

            self._writer_thread = threading.Thread(target=self._write_loop, daemon=True)
            self._writer_thread.start()

            return True

        except serial.SerialException as exc:
            print(f"\n[Fatal] Could not open port {self.port}: {exc}")
            self.running = False
            return False

    def stop_terminal(self) -> None:
        """Stop the background threads and close the port gracefully."""
        if not self.running:
            return

        print("Stopping serial terminal...")
        self.running = False

        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception as exc:  # Serial close is best-effort only.
                print(f"[Warning] Failed to close serial port cleanly: {exc}")

        print("--- Serial Terminal Closed ---")

    def send(self, message: str) -> None:
        """Thread-safe helper that appends a newline and writes to the port."""
        if not self.running or not self.ser or not self.ser.is_open:
            print(f"\n[Warning] Could not send '{message}'. Serial terminal is offline.")
            return

        try:
            with self.write_lock:
                self.ser.write(f"{message}\n".encode("utf-8"))
        except (serial.SerialException, OSError) as exc:
            print(f"\n[Error] Failed to send '{message}': {exc}")
            self.stop_terminal()

    def move_to(self, value_x, value_y) -> None:
        """Send normalized servo positions using the ``x y`` protocol."""
        try:
            value_x = int(value_x)
            value_y = int(value_y)
        except ValueError:
            print(f"\n[Error] move_to: '{value_x}' or '{value_y}' is not a number.")
            return

        if not -1 <= value_x <= 100:
            print(f"\n[Warning] move_to: X value {value_x} is outside 0-100.")
            return
        if not -1 <= value_y <= 100:
            print(f"\n[Warning] move_to: Y value {value_y} is outside 0-100.")
            return

        if value_x == -1 and value_y == -1:
            command = "n n"
        elif value_x == -1:
            command = f"n {value_y}"
        elif value_y == -1:
            command = f"{value_x} n"
        else:
            command = f"{value_x} {value_y}"

        self.send(command)


if __name__ == "__main__":
    port = ArduinoSerialTerminal.scan_for_arduino()

    if not port:
        print("No Arduino detected. Exiting demo.")
    else:
        terminal = ArduinoSerialTerminal(baud_rate=9600)

        if not terminal.start_terminal(port):
            print("Failed to start serial terminal.")
        else:
            print("\n--- Programmatic Control Demo ---")
            print("Interact with the terminal above or press Ctrl+C to exit.")

            try:
                while terminal.running:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                print("\n[Info] Main thread received Ctrl+C. Shutting down...")
                terminal.stop_terminal()

            print("Demo finished.")
