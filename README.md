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
It is off by default; tick it to tune mass, thrust, maximum speed, yaw rate, and
motor response without an MCU. Disable it after connecting the MCU.

Coordinates in the plot are local east/north metres. Yaw is zero at north and
positive clockwise. GPS is selectable at 1/5/10 Hz (default 5 Hz); yaw and serial sensor frames run
at 100 Hz. See [PROTOCOL.md](PROTOCOL.md) for the binary protocol.

Below the map, a telemetry dashboard shows fifteen boxed readouts in a 5 x 3
grid: input source, raw and simulated motor commands, east/north, yaw, speed,
yaw rate, target distance, GPS fix, MCU EKF position, sonar left/right ranges
(`*` marks an echo), motor and EKF frame counters, echo points, and obstacles.
The last column is wider to fit the GPS/EKF coordinates.

Click the map to select a target. The selected local coordinate is converted to
the directly pasteable `sim target <latitude> <longitude>` command and can be
copied to the clipboard or sent to the MCU. The direction mapping controls can
invert the yaw angle or either motor without changing the dynamics model. Motor
channel swapping is enabled by default and can be disabled in the UI.

The true simulated trajectory is blue, 5 Hz GPS samples are orange, and the
EKF coordinates returned by the MCU are shown as a green trace. Motor commands
remain at 100 Hz; EKF position uses a separate `0x82` frame at 1-10 Hz. A 5 Hz
EKF update rate is recommended.

## Camera

The map starts centred on the local origin. Pan the camera by dragging with the
middle or right mouse button (left click keeps picking targets and obstacles).
Enable **Follow boat** in the toolbar to lock the camera on the hull; dragging
the map or pressing **Reset view** leaves follow mode and returns to the origin
at the 80 m view. Zoom with the mouse wheel as before.

## Route planning

The route planner is a separate window, hidden until you open it with the
**Route plan…** button below the manual motor scales; closing it with the X only
hides it and keeps the plan. Click the map and add the selected position to the
route. A route supports up
to 20 ordered waypoints. Starting the route sends waypoint 1 using the existing
target frame. As soon as one EKF position is within the configured arrival
radius, the PC treats that waypoint as reached and automatically sends the next
one. The boat does not need to stop inside the radius. The route is marked
complete after the last waypoint. Stop the route before deleting points;
resetting the simulation does not erase the planned route.

Arrival means the boat remains inside the configured target radius for two
seconds. A 200 ms MCU command timeout stops both simulated motors.

## Distance measure

Pick **Measure** in the map click mode row (active mode is green), then
left-click the map point by point: every segment is labelled with its length
and bearing (north-zero, clockwise, three-digit degrees) and the status line
shows the last segment and the running total. While the measure mode is
active a dashed rubber band follows the cursor with a live length. Clicking
within a few pixels of the first point (from the third point on) closes the
loop and reports the perimeter. Clear with the **Clear measure** button or
`Esc`. Measured points are world coordinates, so panning and zooming keep
them in place; they are PC-side map annotations only.

## Obstacles

In the **Obstacles and shoreline** panel, the map click mode is a row of three
buttons (**Target** / **Stone island** / **Shoreline**); the active mode is
highlighted green. **Generate default shore** drops a random wavy shoreline
across the lower half of the current view and places the boat 0.5-1 m off the
shore, facing open water; clicking it again replaces the previous shore.
Collision response defaults to **Slide along edge** and can be switched to full
stop. Choose **Stone island** and click the map to add circular obstacles.
Choose **Shoreline**, click at least three points, then click **Finish
shoreline** to create a polygon no-go area. The
map keeps the last obstacle at the configured 80 m by 80 m view. Collision
response can stop the boat or remove the velocity component into the obstacle
so it slides along the boundary. These obstacles are PC-side simulation data
and are not included in the serial protocol.
