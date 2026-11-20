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


def matrix_quaternion(matrix):
    """Return xyzw for the configured orthonormal sensor mounting rotation."""
    import numpy as np

    r = np.asarray(matrix, dtype=float)
    q = np.empty(4)
    trace = float(np.trace(r))
    if trace > 0:
        scale = 2 * math.sqrt(1 + trace)
        q[:] = ((r[2,1]-r[1,2])/scale, (r[0,2]-r[2,0])/scale,
                (r[1,0]-r[0,1])/scale, scale/4)
    else:
        i = int(np.argmax(np.diag(r))); j, k = (i+1)%3, (i+2)%3
        scale = 2 * math.sqrt(1+r[i,i]-r[j,j]-r[k,k])
        q[i], q[j], q[k], q[3] = scale/4, (r[i,j]+r[j,i])/scale, (r[i,k]+r[k,i])/scale, (r[k,j]-r[j,k])/scale
    q /= np.linalg.norm(q)
    return tuple(float(v) for v in q)
