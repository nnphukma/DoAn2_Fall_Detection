# Fall Detection data logger (Windows)

Requires Python 3.10+ and Windows Bluetooth LE support. From this directory:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python main.py
```

Flash the ESP32 firmware first, then select **Scan**, choose
`FallDetector_ESP32C3`, and connect. Choose an output folder and filename, then
select **Start Recording**. A CSV and separate `_metadata.json` file are saved
on Stop. Existing CSV files are never overwritten; a numeric suffix is added.

The UI thread never performs BLE I/O. Bleak runs on a background asyncio loop;
notifications are passed through a thread-safe queue and only the Tkinter thread
writes the CSV or updates widgets. If Bluetooth disconnects during a recording,
the application closes and preserves the partial CSV and writes metadata with
`connection_interrupted: true`.

CSV columns are defined in `protocol.py`. FSM state/event are their firmware
numeric enum values; consult `src/modules/fall_detector/fall_detector.h` for the
mapping.
