# Rail IMU

Experimental Android app for ~5 minute inertial velocity/position estimation when the phone is **rigidly fixed to a train/subway car**.

## Agreed design

- No forward-direction calibration.
- No absolute geographic/world frame is required.
- The coordinate frame is frozen at **Start**. Its axes are simply the phone axes at that instant.
- Uses Android hardware `TYPE_ACCELEROMETER` (contains gravity) and `TYPE_GYROSCOPE`.
- Does **not** use Android `TYPE_LINEAR_ACCELERATION`.
- 5 second stationary calibration estimates:
  - measured local gravity magnitude `g_cal`;
  - gravity direction in the installed phone pose;
  - gyroscope static bias.
- During a run, gyro integration tracks the rotation from the current phone frame back to the Start-local frame.
- Raw acceleration is rotated to that fixed local frame, then the calibrated gravity vector is subtracted.
- A gentle 5 Hz first-order low-pass filter is applied before integration.
- Trapezoidal integration estimates 3-D velocity and position; scalar speed, displacement magnitude, and integrated path length are also shown.
- CSV logs raw and derived signals at accelerometer event rate.

## Important estimator choice

The app intentionally does **not** continuously force the accelerometer vector to equal gravity while the train is moving. Longitudinal train acceleration is low-frequency and can be mistaken for a small tilt. An aggressive accelerometer/gyro complementary filter can therefore remove the very acceleration we want to integrate. For this first experiment, the accelerometer is used to establish gravity only while stationary; motion attitude propagation is gyro-driven.

## Build

Recommended environment:

- Android Studio current stable
- JDK 17
- Android SDK 36
- Android Gradle Plugin 8.13.2
- Gradle 8.13

Open this directory in Android Studio, let Gradle sync, then build/install `app`. Alternatively, GitHub Actions in this repository builds a debug APK automatically on every push and can also be run manually.

The execution environment used to generate this project did not contain the Android SDK/Gradle binaries, so a local APK may not be present unless a toolchain became available during generation.

## Usage

1. Rigidly fix the phone to the rail vehicle; any orientation is allowed.
2. While fully stationary, tap **Calibrate 5 s** and do not touch the phone.
3. Tap **Start**.
4. Run for the desired interval (~5 min target).
5. Tap **Stop**.
6. Tap **Export CSV**.

## Interpretation caveat

Pure inertial integration drifts. A single stationary pose can determine the gravity vector in that pose and gyro bias, but it cannot uniquely identify all three accelerometer bias components. The CSV is intentionally detailed so real train data can be used to add bias/zero-velocity corrections later if needed.
