# Differential boat simulator

A desktop closed-loop simulator for an MCU boat controller. The PC simulates
the differential-thrust hull, GPS, and yaw sensor. It sends sensor and target
frames to the MCU and receives left/right motor commands plus the MCU's EKF
position.

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

Click the map to select a target. The selected local coordinate is converted to
the directly pasteable `sim target <latitude> <longitude>` command and can be
copied to the clipboard or sent to the MCU. The
direction mapping controls can invert the gyro rate, invert either motor, or
swap the two motor channels without changing the dynamics model.

The true simulated trajectory is blue, 1 Hz GPS samples are orange, and the
EKF coordinates returned by the MCU are shown as a green trace. Motor commands
remain at 100 Hz; EKF position uses a separate `0x82` frame at 2 Hz or 1 Hz.

Arrival means the boat remains inside the configured target radius for two
seconds. A 200 ms MCU command timeout stops both simulated motors.
