# Mobile Radar Target desktop controller

A Python/PySide6 implementation of `ui-concepts/mobile-radar-target-ui-v3-clean.png`.
It uses the **existing USB dongle and ESP-NOW firmware**. The laptop does not need
an ESP-NOW-capable Wi-Fi adapter. Flashing is a separate Arduino IDE operation. The matched firmware is included under firmware/ in this package.

## Appearance

The app uses one explicit light theme for the main window, Advanced controls and
Qt dialogs, independent of the desktop dark/light setting. Inputs, selections,
disabled controls and dropdowns share the same palette. Fonts use an installed
cross-platform fallback. Operating-system title bars retain their native styling.

## Launch

### Windows

1. Install 64-bit Python 3.12 or 3.13 from [python.org](https://www.python.org/downloads/windows/).
2. Extract the whole ZIP to a writable folder, then double-click
   **Start Cart UI.cmd** at the top level. The first launch uses the internet to
   install the dependencies into a local virtual environment.
3. For a hardware-free check, choose **Demo — no hardware** and click **Connect**.
   For the real cart, plug in the USB dongle and choose its `COM` port.
   If unsure which port to use, compare the list in Device Manager under
   **Ports (COM & LPT)** with the dongle plugged in and unplugged.

To launch Demo or specify a custom recordings folder from PowerShell:

```powershell
& ".\Start Cart UI.cmd" --demo
& ".\Start Cart UI.cmd" --output "C:\Users\YourName\Documents\Cart Runs"
```

### Linux

From the extracted folder or repository root:

```bash
bash "Start Cart UI.sh"
```

The launcher creates a local Python virtual environment and installs the dependencies on first launch. You can also run `bash cart_ui/run.sh`. The UI package contains its protocol helpers.

To explore without hardware:

```bash
bash "Start Cart UI.sh" --demo
```

Click **Connect** with **Demo — no hardware** selected. Demo mode is clearly labelled
and writes to a separate `demo/` directory. It emulates protocol replies and short
runs; its synthetic recordings are **not cart measurements or a dynamics model**.
No serial ports are opened in demo mode.

## Normal workflow

1. Plug the **ESP32-C3 dongle** into the laptop and close any Arduino Serial Monitor
   using it. Select its port (`COMx` on Windows, usually `/dev/ttyACM0` on Linux)
   and click **Connect**. The cart
   remains wireless. Connection requests neutral and waits for an acknowledgement.
2. Edit the waypoint table or **Load CSV**. Examples are in `cart_ui/examples/`.
   Positions are distances from home, in metres; speeds are in m/s.
3. Click **Upload profile**. The app uploads the complete table and checks its CRC,
   point count, launch settings and planned duration. Uploading does not drive.
4. Align the track Hall sensor over the **first/home magnet**, then click **Set home**.
   This declares a position; it does not return the cart automatically. The existing
   firmware map calls this point **0.792 m**; the GUI calls it **0 m**.
5. With the cart powered and the route clear, click **Start run**. Logging is armed
   first (10 Hz by default). Execution stays on the cart; the PC does not stream
   speed targets. There is no automatic motion retry.
6. Wait for the cart and recording to finish, then click **Download recording**.
   Select a parent folder. A unique timestamped folder is created under `hardware/`.
   The download goes through the dongle over ESP-NOW, not cart USB.
7. Set **Advanced → Return cruise speed** as desired, then click **Return home**
   once the forward recording has downloaded. The current valid position
   determines the reverse distance. Return is
   not automatic after a forward run; pressing the button explicitly starts it.
8. Download the return recording, make any small physical alignment adjustment over
   the home magnet, click **Set home**, then repeat. Return home does not redefine
   the cart's position or automatically start another run. After cart/dongle reset
   or reconnection, upload the profile again.

**STOP**, also available with **Escape**, requests neutral immediately and cancels
pending operations. “Acknowledged” means the cart accepted neutral; it may still
coast. A lost connection cannot provide a confirmed stop. Normal application close
requests neutral before closing the port. A forced process kill cannot send STOP;
the dongle generates its own heartbeats, so do not treat a PC crash as a guaranteed
radio-link loss.

There is no live-data dashboard. The small status line reports progress, faults and
recording availability. Buttons are enabled only when their required state is known.

## Profiles and preview

The waypoint table scrolls independently of the Add/Remove buttons. Its vertical
scrollbar is always visible; adding a point selects it and scrolls it into view.


CSV format:

```csv
position_m,speed_mps
0,0
2,2
6,2
8,0
```

The graph uses **linear interpolation of speed squared over position**, matching
the firmware. This produces curved speed ramps rather than the straight lines in
the original mockup. It previews the requested table, not measured motion.
Launch handling, minimum moving speed and final stopping can override the table
near rest. The default launch and moving floor are both **1 m/s**.
In **Advanced**, set the moving floor as low as **0.1 m/s**. Launch speed must
be at least the moving floor and at most the highest waypoint speed. If every
waypoint is below 1 m/s, lower both settings to fit the profile.

The profile editor warns when a nonzero waypoint, moving floor or launch speed
is below **1 m/s**, because the cart can track poorly or move unevenly at these
speeds. The warning leaves valid profiles available for upload and start. Zero
speed at the endpoints is expected. The configured floor can still raise a
requested waypoint speed while the cart is moving.

Existing constraints are retained: 2–32 points, increasing positions, zero speed
at both ends, no interior zero-speed points, up to 10 m/s, and the existing
3.5 m/s² acceleration / 4 m/s² deceleration admission limits. The graph updates when a cell edit is committed (press Enter or click another cell),
even if the requested profile exceeds the firmware limits. Such a profile remains
visible with an explanation, while Upload and Start stay disabled. Invalid or
unordered coordinates show “Preview unavailable” instead of an old curve.
The app rejects an infeasible upload rather than silently adjusting the table.
For an excessive speed change, the warning identifies the segment, required
acceleration/deceleration, configured limit and minimum ramp distance. Hover over
the warning to see all failing segments. In **Advanced**, you can set lower
profile limits for the current UI session. A waypoint table that still fits
the new limits can use its existing upload. If the table needs editing, upload
the revised profile before starting. Raising the firmware ceilings requires
updating the cart firmware and the desktop validator. The current 26 m firmware map
allows an endpoint up to **25.208 m from the first-magnet home**. Physical stopping
and tracking accuracy remain properties of the cart; a valid upload is not a
promise that every requested speed will be reached.

## Return home

The return button sits below the main controls. Hover over a disabled button to see
why it is unavailable. It requires a quiet cart, fresh valid position and a verified
local copy of the current recording. An older download cannot unlock a newer run.
A forward run with logging disabled does not require a recording download.

Set the cruise speed in **Advanced**. The default is 0.75 m/s and the adjustment
range is 0.50–2.00 m/s. Speeds above 1 m/s require both updated firmware sketches
from [firmware/README.md](../firmware/README.md). The cart tapers fast returns toward
0.75 m/s near home and commands neutral early if it enters the final zone too fast.
Short returns may stay below the selected cruise speed.

Home is at 0.792 m in the firmware map. The neutral margin is 12 cm at cruise speeds
up to 0.75 m/s and 20 cm above that. The UI retains current PI gains, records the
return at 10 Hz, and uses the existing 26 m / 60 s bounds. A return requires enough
room to reach the selected cutoff. There are no automatic nudges or retries.

STOP works during preparation and reversing. Faults are reported and their recordings
remain downloadable. A normal return reports **“Return finished—check alignment before
setting home.”** Return recordings have their own timestamped folders and context
(operation, start, goal, cutoff and margin); they are not labelled with the forward
velocity profile.

## Recordings

Click **View tracking** to open the latest automatic or manual download from the
current session. Use **Open recording…** in the viewer to browse older CSVs. You
can also open saved recordings while disconnected. The toolbar lets you zoom,
pan, reset the axes, or save an image.

The viewer plots firmware setpoint and measured speed against time, along with
tracking error, logged throttle and speed against corrected position when those
channels are available. When the downloaded `recording.csv` has a matching
`profile.json` in its folder, the distance plot also shows the waypoint profile.

Each download folder contains:

- `recording.csv`: the actual firmware sample columns and recorded timestamps.
- `recording.metadata.json`: firmware/calibration information, transport verification,
  stop reason, recording flags and RAM release status where available.
- `profile.json`: the profile for a run started in this app, when applicable.
- `context.json`: command, gains, logging rate and time for an app-started run.
- `transfer.log`: the raw recording transfer.
- `commands.log`: the serial session log, including command acknowledgements.

Transfers are verified before output files are created. All files are closed and
flushed before releasing the cart's frozen RAM recording. Failed/cancelled downloads
do not release it. Saved flash recordings are downloaded by ID and are not erased.
Faulted runs are retained for diagnosis; an endpoint error is not relabelled a success.
If the app is reopened to retrieve an older recording, its CSV/metadata are retained
without inventing a missing profile or command context.

The default output folder is `MobileRadarTarget` under your system's Documents folder. Override with
`--output /your/folder`. Keep the cart powered until a RAM recording has been saved.

## Advanced controls

The separate Advanced window includes:

- Read/apply **Kp and Ki**, with readback. Changes are RAM-only and are lost on reset.
  Opening the panel reads current gains when the cart is stationary and available.
- Launch speed and minimum moving speed. Changes require a new profile upload.
- Profile max acceleration and deceleration. These validate the nominal ramps
  between waypoints. Launch, minimum speed, stopping and actual cart motion
  still follow the cart controller.
- Logging at **10 Hz**, **50 Hz**, or **Off** for the next run. “Disable logging now”
  is stopped-only and cannot discard an undownloaded frozen recording.
- Fixed-throttle tests: signed commands up to 100%, 0.05–2 seconds.
- Timed-speed tests: signed targets up to 10 m/s, 0.05–3 seconds. The start
  confirmation warns if the target magnitude is above zero and below 1 m/s.
- Diagnostic messages and a second STOP button.

Advanced tests retain the firmware's distance guards (25 m powered travel for fixed
throttle; 15 m for timed speed). These exclude subsequent braking travel. No limits
are increased by the UI. 50 Hz can fill the 896-sample buffer on long profiles;
the app rejects profiles whose estimated recording budget is too large at that rate.
Select Off to remove logging as a variable during debugging.

## Verification and current limits

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest cart_ui/tests -q
```

Tests exercise the basic GUI sequence through the simulated serial protocol,
validation, readbacks, download integrity, disk failures, missing acknowledgements,
STOP preemption, recording-specific return gating and the complete forward/download/return/download/realign cycle. Generate an offline screenshot with:

```bash
QT_QPA_PLATFORM=offscreen python3 -m cart_ui.app --screenshot /tmp/cart-ui.png
```

**Physical board validation is pending.** No cart/dongle USB ports were present
when this app was implemented. Before the first powered GUI run, with motor power
disconnected, check Connect, profile upload, Set home, PI readback, STOP and recording
retrieval on the actual boards. Use the matched firmware in this package for adjustable returns above 1 m/s. Reconcile any older firmware with the capability checks before using that feature.
Do not issue Start, Return home or timed tests as part of unattended validation.
The integrated software is ready for one supervised physical cycle; actual reverse
stopping accuracy and wireless operation of the complete GUI still need that check.

Implementation: `backend.py` owns command sequencing and storage; `protocol.py`
handles wire-format validation and parsing; `worker.py` owns
the sole serial connection on a background thread; `app.py` contains the UI;
`simulator.py` provides an explicitly selected hardware-free test connection.
