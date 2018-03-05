from ubitrack.core import math as utmath
from ubitrack.core import measurement
import numpy as np
import math

def test_measurement_button():
    "test measurement button"

    i = measurement.Button()
    assert i.invalid()
    assert i.get() == None
    
    b = measurement.Button(123, utmath.ScalarInt(1))
    assert b.get().value == 1
    assert b.time() == 123
    assert b.invalid() == False
    
    b.invalidate()
    assert b.invalid()
    
def test_measurement_pos2d():
    "test measurement Position2D"

    i = measurement.Position2D()
    assert i.invalid()
    assert i.get() == None
    
    b = measurement.Position2D(123, utmath.Vector2d(1.0,2.0))
    print(b.get())
    print(np.array([1.0,2.0]))
    assert np.all(b.get() == np.array([1.0,2.0]))
    assert b.time() == 123
    assert b.invalid() == False
    
def test_measurement_pos3d():
    "test measurement Position(3D)"

    i = measurement.Position()
    assert i.invalid()
    assert i.get() == None
    
    b = measurement.Position(123, utmath.Vector3d(1.0, 2.0, 3.0))
    assert np.all(b.get() == np.array([1.0, 2.0, 3.0]))
    assert b.time() == 123
    assert b.invalid() == False
    
def test_measurement_matrix3x3():
    "test measurement Matrix3x3"

    i = measurement.Matrix3x3()
    assert i.invalid()
    assert i.get() == None
    m = np.array([[1.0, 2.0, 3.0],[1.0, 2.0, 3.0],[1.0, 2.0, 3.0]])
    b = measurement.Matrix3x3(123, utmath.Matrix33d(m))
    print(b.get())
    print(m)
    assert np.all(b.get() == m)
    assert b.time() == 123
    assert b.invalid() == False
    
def test_measurement_rotation():
    "test measurement Rotation"

    i = measurement.Rotation()
    assert i.invalid()
    assert i.get() == None

    q = utmath.Quaternion()
    b = measurement.Rotation(123, q)
    quat = b.get()
    assert np.all(quat.toVector() == q.toVector())
    assert b.time() == 123
    assert b.invalid() == False
    
def test_measurement_pose():
    "test measurement Pose"

    i = measurement.Pose()
    assert i.invalid()
    assert i.get() == None

    m = np.array([[1.,2.,3.,4.],[1.,2.,3.,4.],[1.,2.,3.,4.],[1.,2.,3.,4.]])
    p = utmath.Pose(m)
    b = measurement.Pose(123, p)
    p1 = b.get()
    assert np.all(p1.toVector() == p.toVector())
    assert b.time() == 123
    assert b.invalid() == False
    
def test_measurement_poselist():
    "test measurement poselist"

    poses = utmath.PoseList()
    for i in range(5):
        poses.push_back(utmath.Pose(utmath.Quaternion(0.0, 0.0, 0.0, 1.0), utmath.Vector3d(1.0, 2.0, 3.0)))

    m = measurement.PoseList(measurement.now(), poses)
    result = m.get()
    assert len(result) == 5
    
    element = result[0]
    assert np.all(element.translation() == np.array([1.0, 2.0, 3.0])) 
    
