---
description: Outdoor RTK and IMU localization; optional inertial error-state fusion, no device I/O.
---

Consumes GNSS fix/rtk_status/velocity and IMU imu/angular_velocity/status through Atlas.
`estimator_mode=eskf` additionally requires primitive/imu/inertial and confirmed magnetic heading quality.
Outputs existing odom, pose, TF and JSON status. No chassis odometry, serial, NTRIP or command feedback.

ESKF: 3D acceleration/gyro propagation, horizontal GNSS updates, attitude/bias estimation, 2s delayed replay.
Initialization requires 5s stationary IMU and 2s usable RTK. No RTK: continue inertial estimates with growing
uncertainty; navigation readiness remains separate. Fixed recovery reanchors horizontal position/velocity,
retaining attitude/bias and continuous local odom. No geographic altitude is advertised.

Status adds fusion stage, estimate_valid, navigation_ready, uncertainty, outage_s, recovery_id and diagnostics.
`fusion.rtk_grade` reports the fresh receiver fixed/float state independently of filter initialization.
`inertial_elapsed_s` measures the current outage; `fusion_elapsed_s` resets on each accepted fixed fix,
or starts at initialization if none has been accepted. Scene displays and records these fields.
`grade` can be fixed, float, inertial or null. Navigation limits default to horizontal 2σ=3m, yaw 2σ=30°.
Noise defaults and reported covariance require field validation; static initialization is not full calibration.
