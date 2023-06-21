from unittest import TestCase
import numpy as np
from SfM.camera import Camera
from scipy.spatial.transform import Rotation as R

class TestCamera(TestCase):
    def test_get_viewing_line(self):
        trans = np.array([1, 2, 3])
        rot = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        camera = Camera(translation=trans, rotation=R.from_matrix(rot))
        line = camera.get_viewing_line()
        np.testing.assert_equal(line.point3D.get_np(), trans)
        np.testing.assert_equal(line.vector3D.get_np(), rot[2, :])
