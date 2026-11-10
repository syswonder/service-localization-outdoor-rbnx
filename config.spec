specVersion: 1
required: [origin]
properties:
  origin:
    type: object
    description: Explicit fixed WGS84 map datum with latitude and longitude. Never inferred from the first fix.
  require_map_reference:
    type: boolean
    default: false
    description: Suppress odometry and TF until a saved map identity is supplied.
  map_id:
    type: string
    description: Loaded immutable outdoor map ID, supplied together with map_version.
  map_version:
    type: string
    description: Loaded map content hash. Scene reinitializes the existing Driver with its origin and identity.
  provider_ids:
    type: object
    properties:
      gnss:
        type: string
        x-provider: robonix/primitive/gnss/fix
      imu:
        type: string
        x-provider: robonix/primitive/imu/imu
  topic_remap:
    type: object
    description: Explicit test/legacy bindings for gnss_fix,rtk_status,gnss_velocity,imu,gyro,imu_status; otherwise resolved through Atlas.
  imu:
    type: object
    description: Measured mounting rotation, north correction, yaw uncertainty, timeout and confirmations. See configuration.py.
  output_topics:
    type: object
    description: Odometry, map pose, readiness JSON, dynamic TF and TF-static topic names.
  velocity_accuracy_mps:
    type: number
    description: Independently verified bound. RMC has no velocity uncertainty metadata; absent by default.
  velocity_accuracy_confirmed:
    type: boolean
    default: false
  calibrated:
    type: boolean
    default: false
  antenna_offset_confirmed:
    type: boolean
    default: false
  feedback_confirmed:
    type: boolean
    default: false
  estimator_mode:
    type: string
    default: legacy
    description: legacy receiver-velocity mode or eskf inertial fusion; eskf requires the inertial primitive capability.
  fusion:
    type: object
    description: Static initialization (5s), replay history (at most 2s), IMU position in base axes, noise densities/bias random walks and navigation uncertainty limits; see configuration.py.
