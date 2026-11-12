from __future__ import annotations

import asyncio
import csv
import json
import os
import queue
import threading
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from bleak import BleakClient, BleakScanner
from protocol import CSV_COLUMNS, Reassembler, decode_packet

DEVICE_NAME = "FallDetector_ESP32C3"
SERVICE_UUID = "7c4c1000-8a3f-4c7d-a1f5-9b7f0c000001"
DATA_UUID = "7c4c1001-8a3f-4c7d-a1f5-9b7f0c000001"
DEFAULT_OUTPUT_DIRECTORY = Path(r"D:\VDK\ESP32\ESP32C3\ESP32C3_MachPhatHienNga\FallDectiorData")

class BleWorker:
    def __init__(self, events: queue.Queue):
        self.events, self.loop, self.client = events, None, None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        self.loop = asyncio.new_event_loop(); asyncio.set_event_loop(self.loop); self.loop.run_forever()

    def submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    async def scan(self):
        self.events.put(("status", "Scanning"))
        advertisements = await BleakScanner.discover(timeout=5.0, return_adv=True)
        devices = []
        for device, advertisement in advertisements.values():
            name = advertisement.local_name or device.name or "Unnamed"
            service_uuids = [uuid.lower() for uuid in (advertisement.service_uuids or [])]
            is_fall_detector = SERVICE_UUID.lower() in service_uuids
            label = f"{name} [Fall Detector]" if is_fall_detector else name
            devices.append((label, device.address))
        self.events.put(("devices", devices))
        self.events.put(("status", "Disconnected"))

    async def connect(self, address):
        self.events.put(("status", "Connecting"))
        try:
            self.client = BleakClient(address, disconnected_callback=lambda _: self.events.put(("lost", None)))
            await self.client.connect()
            if self.client.services.get_service(SERVICE_UUID) is None:
                raise RuntimeError("Thiết bị không có Fall Detector BLE service")
            reassembler = Reassembler()
            def notification(_, data):
                try:
                    packet = reassembler.feed(data)
                    if packet: self.events.put(("sample", decode_packet(packet)))
                except ValueError as exc: self.events.put(("invalid", str(exc)))
            await self.client.start_notify(DATA_UUID, notification)
            self.events.put(("status", "Connected"))
        except Exception as exc:
            if self.client and self.client.is_connected:
                await self.client.disconnect()
            self.client = None
            self.events.put(("error", str(exc)))
            self.events.put(("status", "Disconnected"))

    async def disconnect(self):
        if self.client:
            if self.client.is_connected: await self.client.disconnect()
            self.client = None
        self.events.put(("status", "Disconnected"))

class App(ttk.Frame):
    def __init__(self, root):
        super().__init__(root, padding=10); self.pack(fill="both", expand=True)
        self.root = root
        self.events = queue.Queue()
        self.worker = BleWorker(self.events)
        self.connected = self.recording = self.connection_lost = False
        self.csv_file = self.writer = self.start_time = self.last_sequence = None
        self.samples = self.missing = self.invalid = 0; self.rate_window = []
        self.path = DEFAULT_OUTPUT_DIRECTORY; self.path.mkdir(parents=True, exist_ok=True)
        self.filename = tk.StringVar(value=f"fall_test_{datetime.now():%Y%m%d_%H%M%S}.csv")
        self.status = tk.StringVar(value="Disconnected"); self.stats = {key: tk.StringVar(value="—") for key in ("elapsed", "samples", "rate", "loss", "timestamp", "state", "event")}
        self._build(); self.after(100, self._poll)

    def _build(self):
        self.root.title("Fall Detection – Data Logger"); self.columnconfigure(0, weight=1)
        ble = ttk.LabelFrame(self, text="BLE connection", padding=8); ble.grid(sticky="ew"); ble.columnconfigure(0, weight=1)
        self.devices = tk.Listbox(ble, height=4); self.devices.grid(row=0, column=0, columnspan=3, sticky="ew")
        ttk.Button(ble, text="Scan", command=lambda: self.worker.submit(self.worker.scan())).grid(row=1, column=0)
        ttk.Button(ble, text="Connect", command=self.connect).grid(row=1, column=1)
        ttk.Button(ble, text="Disconnect", command=lambda: self.worker.submit(self.worker.disconnect())).grid(row=1, column=2)
        ttk.Label(ble, textvariable=self.status).grid(row=1, column=3, padx=12)
        cfg = ttk.LabelFrame(self, text="Recording configuration", padding=8); cfg.grid(sticky="ew", pady=8); cfg.columnconfigure(0, weight=1)
        self.dir_label = ttk.Label(cfg, text=str(self.path)); self.dir_label.grid(row=0, column=0, sticky="w")
        ttk.Button(cfg, text="Browse…", command=self.browse).grid(row=0, column=1)
        ttk.Entry(cfg, textvariable=self.filename).grid(row=1, column=0, columnspan=2, sticky="ew", pady=5)
        self.start_button = ttk.Button(cfg, text="Start Recording", command=self.start_recording); self.start_button.grid(row=2, column=0, sticky="ew")
        ttk.Button(cfg, text="Stop Recording", command=lambda: self.stop_recording()).grid(row=2, column=1, sticky="ew")
        live = ttk.LabelFrame(self, text="Live recording", padding=8); live.grid(sticky="ew");
        for i, (label, key) in enumerate((("Elapsed time", "elapsed"), ("Received samples", "samples"), ("Reception rate", "rate"), ("Packet loss", "loss"), ("Latest timestamp", "timestamp"), ("FSM state", "state"), ("Latest event", "event"))):
            ttk.Label(live, text=label).grid(row=i//2, column=(i%2)*2, sticky="w", padx=(0, 8)); ttk.Label(live, textvariable=self.stats[key]).grid(row=i//2, column=(i%2)*2+1, sticky="w", padx=(0, 20))

    def browse(self):
        chosen = filedialog.askdirectory(initialdir=self.path)
        if chosen: self.path = Path(chosen); self.dir_label.config(text=str(self.path))
    def connect(self):
        selection = self.devices.curselection()
        if not selection: return messagebox.showwarning("Select device", "Select FallDetector_ESP32C3 first.")
        self.worker.submit(self.worker.connect(self.devices.get(selection[0]).split(" | ")[-1]))
    def _target(self):
        name = self.filename.get().strip() or f"fall_test_{datetime.now():%Y%m%d_%H%M%S}.csv"
        if not name.lower().endswith(".csv"): name += ".csv"
        path = self.path / name; n = 1
        while path.exists(): path = self.path / f"{Path(name).stem}_{n}{Path(name).suffix}"; n += 1
        return path
    def start_recording(self):
        if not self.connected: return messagebox.showwarning("Not connected", "Connect to the ESP32-C3 before recording.")
        self.path.mkdir(parents=True, exist_ok=True); self.output = self._target()
        self.csv_file = self.output.open("w", newline="", encoding="utf-8"); self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_COLUMNS); self.writer.writeheader()
        self.recording = True; self.connection_lost = False; self.start_time = time.monotonic(); self.start_wall = datetime.now(); self.samples = self.missing = 0; self.last_sequence = None
        self.status.set("Recording")
    def stop_recording(self, lost=False):
        if not self.recording: return
        self.recording = False; self.connection_lost |= lost; self.csv_file.flush(); self.csv_file.close()
        duration = time.monotonic() - self.start_time; loss = self.missing / (self.samples + self.missing) * 100 if self.samples + self.missing else 0
        metadata = {"recording_start_local": self.start_wall.isoformat(), "recording_stop_local": datetime.now().isoformat(), "recording_duration_s": duration, "device_name": DEVICE_NAME, "protocol_version": 1, "expected_sample_rate_hz": 100, "received_samples": self.samples, "missing_packets": self.missing, "packet_loss_percent": loss, "connection_interrupted": self.connection_lost}
        self.output.with_name(self.output.stem + "_metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        self.status.set("Connection lost" if lost else "Connected")
        messagebox.showinfo("Recording summary", f"Saved: {self.output}\nDuration: {duration:.1f} s\nSamples: {self.samples}\nMissing: {self.missing} ({loss:.2f}%)")
    def _sample(self, row):
        if self.last_sequence is not None:
            gap = (row["sequence"] - self.last_sequence - 1) & 0xFFFFFFFF
            if gap == 0xFFFFFFFF: return # duplicate
            self.missing += gap
        self.last_sequence = row["sequence"]; self.samples += 1; self.rate_window.append(time.monotonic()); self.rate_window = [t for t in self.rate_window if time.monotonic() - t <= 1]
        if self.recording: self.writer.writerow(row)
        self.stats["samples"].set(str(self.samples)); self.stats["rate"].set(f"{len(self.rate_window)} / 100 Hz"); self.stats["loss"].set(f"{self.missing} ({self.missing / (self.samples + self.missing) * 100:.2f}%)" if self.samples + self.missing else "0")
        self.stats["timestamp"].set(str(row["timestamp_ms"])); self.stats["state"].set(str(row["fsm_state"])); self.stats["event"].set(str(row["fsm_event"]))
    def _poll(self):
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "status": self.status.set(value); self.connected = value == "Connected"
                elif kind == "devices":
                    # Windows can report a cached/empty local name even when a
                    # device is advertising. Show every result for diagnosis;
                    # the expected ESP32 name is FallDetector_ESP32C3.
                    self.devices.delete(0, tk.END)
                    for name, address in value:
                        self.devices.insert(tk.END, f"{name} | {address}")
                elif kind == "sample": self._sample(value)
                elif kind == "lost": self.connected = False; self.stop_recording(lost=True); self.status.set("Connection lost")
                elif kind == "invalid": self.invalid += 1
                elif kind == "error": messagebox.showerror("Không thể kết nối", value)
        except queue.Empty: pass
        if self.recording: self.stats["elapsed"].set(time.strftime("%H:%M:%S", time.gmtime(time.monotonic() - self.start_time)))
        self.after(100, self._poll)

if __name__ == "__main__":
    root = tk.Tk(); App(root); root.mainloop()
