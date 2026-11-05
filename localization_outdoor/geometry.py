import math


def local_radii(latitude):
    a = 6378137.0
    eccentricity_sq = 6.69437999014e-3
    sin_lat = math.sin(math.radians(latitude))
    denominator = math.sqrt(1 - eccentricity_sq * sin_lat * sin_lat)
    east_radius = a / denominator * math.cos(math.radians(latitude))
    north_radius = a * (1 - eccentricity_sq) / denominator**3
    return east_radius, north_radius


def enu(latitude, longitude, anchor):
    east_radius, north_radius = local_radii(anchor["latitude"])
    return (
        math.radians(longitude - anchor["longitude"]) * east_radius,
        math.radians(latitude - anchor["latitude"]) * north_radius,
    )


def quaternion_matrix(x, y, z, w):
    """Convert a unit quaternion into its 3D rotation matrix."""
    import numpy as np

    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def wrap(v):
    return math.atan2(math.sin(v), math.cos(v))
