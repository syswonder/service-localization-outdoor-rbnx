---
description: Outdoor localization from GNSS and IMU primitive measurements; no device I/O.
---
# Localization capability

Consumes one GNSS provider for fix/rtk_status/velocity and one IMU provider for imu/angular_velocity/status through Atlas.
Provides robonix/service/localization/{odom,pose,status}; owns map→odom and odom→base_link TF only.
No serial, NTRIP, receiver protocol parsing, acceleration integration, command feedback or chassis odometry.
Origin and mounting/north/uncertainty calibration are explicit deployment inputs. Unknown covariance, unconfirmed accuracy or stale data never means ready.
The package uses an explicit namespace Driver for the SDK validated by this repository; do not also advertise a second shared Driver.
