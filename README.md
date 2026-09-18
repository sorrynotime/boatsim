# Differential boat simulator

A desktop closed-loop simulator for an MCU boat controller. The PC simulates
the differential-thrust hull, GPS, and yaw sensor. It sends sensor and target
frames to the MCU and receives left/right motor commands.

## Run

Python 3.11 or newer is recommended.

```powershell
python -m pip install -r requirements.txt
python app.py
```

The application also works without `pyserial` in **Local manual motor** mode.
Use that mode first to tune mass, thrust, maximum speed, yaw rate, and motor
response. Disable it after connecting the MCU.

Coordinates in the plot are local east/north metres. Yaw is zero at north and
positive clockwise. GPS is generated at 1 Hz; yaw and serial sensor frames run
at 100 Hz. See [PROTOCOL.md](PROTOCOL.md) for the binary protocol.

Arrival means the boat remains inside the configured target radius for two
seconds. A 200 ms MCU command timeout stops both simulated motors.
