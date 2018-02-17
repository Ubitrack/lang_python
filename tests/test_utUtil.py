__author__ = 'jack'

from nose import with_setup
import os
from ubitrack.core import math as utmath, measurement, util
import numpy as np
import math
import tempfile

import unittest


tmpfile = None

def setup_func():
    "set up test fixtures"
    global tmpfile
    tmpfile = tempfile.mktemp(".calib")
    print tmpfile

def teardown_func():
    "tear down test fixtures"
    global tmpfile
    # if os.path.isfile(tmpfile):
    #     os.unlink(tmpfile)
    tmpfile = None




@with_setup(setup_func, teardown_func)
def test_basic_datatypes():
    "test basic data types: Vector2-8, Matrix33-44, Pose, Quaternion"

    ts = measurement.now()

    p = measurement.Position(ts, np.array([1.0, 2.0, 3.0]))
    util.writeCalibMeasurementPosition(tmpfile, p)
    pr = util.readCalibMeasurementPosition(tmpfile)
    assert np.all(p.get() == pr)

    p = measurement.Rotation(ts, utmath.Quaternion())
    util.writeCalibMeasurementRotation(tmpfile, p)
    pr = util.readCalibMeasurementRotation(tmpfile)
    assert p.get() == pr


@with_setup(setup_func, teardown_func)
def test_positionrecord():
    "test reading a position3d recording"
    sb = util.streambuf(open("position_record_test.log", "r"), 1024)
    st = util.PositionStreamReader(sb)
    v = st.values()
    for i in v:
        print i
        p = i.get()
        print p
        print i.time()


def disabled_t_e_s_t_s():
    return
    assert b.value == 1

    d = utmath.ScalarDouble(1.0)
    assert d.value == 1.0

    q = utmath.test_quat()
    assert q.x() == 0 and q.y() == 0 and q.z() == 0 and q.w() == 1

    m = utmath.test_mat33()
    print m
    print np.identity(3)
    assert np.all(m == np.identity(3))

    # from vector
    p = utmath.Pose(q, m[0,:])
    q1 = utmath.Quaternion(m[0,:], 1.0)

    # from matrix
    m1 = np.identity(4)
    p1 = utmath.Pose(m1)

    #test accessors
    assert np.all(p1.translation() == np.asarray([0,0,0]))
    assert np.all(p1.rotation().toVector() == np.array([0,0,0,-1]))

    # needs proper verification 
    p2 = p * p1
    p3 = p.invert()
    
    v = p3 * np.asarray([1.,2.,3.])
    



# class test_quaternion( unittest.TestCase):
#
#
#     def setUp( self ):
#         pass
#
#     def tearDown( self ):
#         pass
#
#     def test_create_identity( self ):
#         result = utmath.Quaternion().toVector()
#
#         expected = np.array( [ 0.0, 0.0, 0.0, 1.0 ] )
#
#         self.assertTrue(
#             np.array_equal( result, expected ),
#             "Quaternion identity incorrect"
#             )
#
#     def test_create_quaternion(self):
#         m = np.array([[1.,2.,3.,4.],[1.,2.,3.,4.],[1.,2.,3.,4.],[1.,2.,3.,4.]])
#         p = utmath.Pose(m)
#         axis, angle = p.rotation().toAxisAngle()
#
#         q = utmath.Quaternion(axis, angle)
#         self.assertTrue(str(q) == str(p.rotation()),
#                         "quaternion created from axis angle is not equal to the original")
#
#
#         #axis angle initialization not working
#         v = np.array([1,0,0])
#         q = utmath.Quaternion(v, 0.)
#         print q.toVector()
#         self.assertTrue(np.all(q.toVector() == np.array([0.,0.,0.,1.,])),
#                         "quaternion created from axis angle has wrong data")
#
#
#     def test_normalise( self ):
#         def identity():
#             # normalise an identity quaternion
#             quat = utmath.Quaternion()
#             quat.normalize()
#
#             expected = np.array( [ 0.0, 0.0, 0.0, 1.0 ] )
# #             assert np.array_equal(
# #                 expected,
# #                 quat / math.sqrt( np.sum( quat ** 2 ) )
# #                 )
#
#             self.assertTrue(
#                 np.array_equal( quat.toVector(), expected ),
#                 "Normalise identity quaternion incorrect"
#                 )
#         identity()
#
#         def non_identity():
#             # normalise a quaternion of length 2.0
#             quat = np.array( [ 1.0, 2.0, 3.0, 4.0 ] )
#             result = utmath.Quaternion.fromVector(quat)
#             result.normalize()
#
#             expected = quat / math.sqrt( np.sum( quat ** 2 ) )
#
#             # check the length is 1.0
#             self.assertTrue(
#                 np.array_equal( result.toVector(), expected ),
#                 "Normalise quaternion incorrect"
#                 )
#         non_identity()
#
#     def test_length( self ):
#         def identity():
#             quat = utmath.Quaternion()
#             result = utmath.abs(quat)
#
#             expected = 1.0
#
#             self.assertEqual(
#                 result,
#                 expected,
#                 "Identity quaternion length calculation incorrect"
#                 )
#         identity()
#
#         def non_identity():
#             quat = np.array( [ 1.0, 2.0, 3.0, 4.0 ] )
#             result = utmath.abs(utmath.Quaternion.fromVector(quat))
#
#             expected = math.sqrt( np.sum( quat ** 2 ) )
#             self.assertEqual(
#                 result,
#                 expected,
#                 "Quaternion length calculation incorrect"
#                 )
#         non_identity()
#
#
#     def test_apply_to_vector( self ):
#         # Euler angle constructor uses X-Z-Y ordering
#
#         def identity():
#             quat = utmath.Quaternion()
#             vec = np.array( [ 1.0, 0.0, 0.0 ] )
#
#             result = quat.transformVector(vec)
#
#             expected = vec
#
#             self.assertTrue(
#                 np.array_equal( result, expected ),
#                 "Quaternion apply_to_vector incorrect with identity"
#                 )
#         identity()
#
#         def rotated_x():
#             quat = utmath.Quaternion(  0.0, 0.0, math.pi )
#             vec = np.array( [ 0.0, 1.0, 0.0 ] )
#
#             result = quat.transformVector(vec)
#
#             expected = -vec
#
#             self.assertTrue(
#                 np.allclose( result, expected ),
#                 "Quaternion apply_to_vector incorrect with rotation about X"
#                 )
#         rotated_x()
#
#         def rotated_y():
#             quat = utmath.Quaternion( 0.0, math.pi, 0.0 )
#             vec = np.array( [ 1.0, 0.0, 0.0 ] )
#
#             result = quat.transformVector(vec)
#
#             expected = -vec
#
#             self.assertTrue(
#                 np.allclose( result, expected ),
#                 "Quaternion apply_to_vector incorrect with rotation about Y"
#                 )
#         rotated_y()
#
#         def rotated_z():
#             quat = utmath.Quaternion( math.pi, 0.0, 0.0 )
#             vec = np.array( [ 1.0, 0.0, 0.0 ] )
#
#             result = quat.transformVector(vec)
#
#             expected = -vec
#
#             self.assertTrue(
#                 np.allclose( result, expected ),
#                 "Quaternion apply_to_vector incorrect with rotation about Y"
#                 )
#         rotated_z()
