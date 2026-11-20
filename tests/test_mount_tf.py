import numpy as np
import pytest
from localization_outdoor.geometry import matrix_quaternion, quaternion_matrix

@pytest.mark.parametrize("matrix",[np.eye(3),np.array([[0,-1,0],[1,0,0],[0,0,1]]),np.diag([1,-1,-1]),np.diag([-1,1,-1])])
def test_mount_rotation_round_trip(matrix):
    q=matrix_quaternion(matrix)
    assert np.linalg.norm(q)==pytest.approx(1.)
    assert np.allclose(quaternion_matrix(*q),matrix)
