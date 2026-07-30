# Drivetrain Simulator

Single-file C++ simulation of a rear-wheel-drive car powertrain:

```
Engine (torque curve) -> Clutch -> 3-speed Gearbox -> Final drive
   -> Open differential -> 2 rear wheels (Pacejka tires) -> Chassis
   -> load + speed feed back up the chain to the engine
```

Visualized with **raylib** (gauges, car, live plots) and **raygui** (parameter
sliders). The whole project is `main.cpp` + `CMakeLists.txt`; the deps are
pulled automatically by CMake's FetchContent.

## Build

Linux needs the usual OpenGL/X11 dev headers first:

```bash
sudo apt-get install -y build-essential cmake git \
    libgl1-mesa-dev libx11-dev libxrandr-dev \
    libxinerama-dev libxcursor-dev libxi-dev
```

macOS needs only Xcode command-line tools + cmake (frameworks are linked by the
CMake file). Then everywhere:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
./build/carsim
```

The first configure spends a couple of minutes compiling raylib from source.

## Controls

| Key        | Action                         |
|------------|--------------------------------|
| W / Up     | throttle                       |
| S / Down   | brake                          |
| Space      | disengage clutch (hold)        |
| Q / E      | shift down / up (N,1,2,3)      |
| 1 / 2 / 3  | surface: dry / wet / ice       |
| R          | reset                          |

The sliders on the left do the same things if you prefer the mouse.

## What to watch for (ties back to the physics)

- **Launch:** in gear 1, feed throttle and ease the clutch up from 0. The
  `engine rpm` and `wheel-equiv rpm` traces start far apart (clutch slipping)
  and converge as it soft-locks. The clutch is what bridges idle-rpm engine to
  zero-rpm wheels.
- **Wheelspin on ice:** press `3`, then floor it. Rear slip `k` shoots past the
  green `k_peak` lines, the wheels flash red, and the green **force** plot
  *collapses* — more throttle, less force. Feather the throttle instead and slip
  stays under `k_peak`, giving more grip. Note `k_peak` itself is much smaller on
  ice than dry (printed on the left), so the stable window is razor-thin.
- **Reflected inertia** uses ratio-squared in the lock coupling; **driveline
  efficiency** (0.90) and **weight transfer** (rear gains load under accel,
  raising rear grip) are both modeled.
- **Tire** is the Pacejka magic formula with `D = mu * Fz`, and `B` rebuilt from
  a fixed slip stiffness so the peak shifts left as the surface gets slippery.
  Low speed uses a relaxation-length slip ODE so launches aren't singular.

## Where the feedback loop lives in the code

`sim.h` is a backend-agnostic API (`namespace vsim`): a host feeds a
`vsim::StepInput` (driver `Command` + one `ContactIn` per wheel) into a
`vsim::IVehicleSim` and reads back `wheelOutputs()` and `telemetry()`. The Ford
Transit manual drivetrain is one implementation of it, `vsim::ManualDrivetrain`
in `drivetrain.h`/`drivetrain.cpp`; swapping in an EV, an automatic or a
different physics backend means writing another `IVehicleSim`, not touching the
front-ends.

In `ManualDrivetrain::integrate()`:
- `omegaIn = omegaCarrier * n;`  -> wheel speed reflected **back up** to the engine.
- `Treact = wh.Fx * wh.r;`       -> tire load reflected **back up** the chain.

Those two are the "back to the engine" path.
