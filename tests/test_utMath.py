__author__ = 'jack'

from nose import with_setup

from ubitrack.core import math as utmath
import numpy as np
import math

try:
    import unittest2 as unittest
except:
    import unittest


def setup_func():
    "set up test fixtures"
    pass

def teardown_func():
    "tear down test fixtures"
    pass

@with_setup(setup_func, teardown_func)
def test_basic_datatypes():
    "test basic data types: Vector2-8, Matrix33-44, Pose, Quaternion"

    b = utmath.ScalarInt(1)
    assert b.value == 1
    
    d = utmath.ScalarDouble(1.0)
    assert d.value == 1.0

    q = utmath.Quaternion()
    assert q.x() == 0 and q.y() == 0 and q.z() == 0 and q.w() == 1

    m = utmath.Matrix33d()
    print(m)
    print(np.identity(3))
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
    
    v = p3 * utmath.Vector3d(1.,2.,3.)
    



class test_quaternion_simple(unittest.TestCase):


    def setUp( self ):
        pass

    def tearDown( self ):
        pass

    def test_create_identity( self ):
        result = utmath.Quaternion().toVector()

        expected = np.array( [ 0.0, 0.0, 0.0, 1.0 ] )

        self.assertTrue(
            np.array_equal( result, expected ),
            "Quaternion identity incorrect"
            )

    def test_create_quaternion(self):
        m = np.array([[1.,2.,3.,4.],[1.,2.,3.,4.],[1.,2.,3.,4.],[1.,2.,3.,4.]])
        p = utmath.Pose(m)
        axis, angle = p.rotation().toAxisAngle()
        
        q = utmath.Quaternion(axis, angle)
        self.assertTrue(str(q) == str(p.rotation()),
                        "quaternion created from axis angle is not equal to the original")


        #axis angle initialization not working 
        v = np.array([1.,0.,0.])
        q = utmath.Quaternion(v, 0.)
        print(q.toVector())
        self.assertTrue(np.all(q.toVector() == np.array([0.,0.,0.,1.,])),
                        "quaternion created from axis angle has wrong data")


    def test_normalise( self ):
        def identity():
            # normalise an identity quaternion
            quat = utmath.Quaternion()
            quat.normalize()

            expected = np.array( [ 0.0, 0.0, 0.0, 1.0 ] )
#             assert np.array_equal(
#                 expected,
#                 quat / math.sqrt( np.sum( quat ** 2 ) )
#                 )

            self.assertTrue(
                np.array_equal( quat.toVector(), expected ),
                "Normalise identity quaternion incorrect"
                )
        identity()
    
        def non_identity():
            # normalise a quaternion of length 2.0
            quat = np.asarray([1.0, 2.0, 3.0, 4.0])
            result = utmath.Quaternion.fromVector(quat)
            result.normalize()

            expected = quat / math.sqrt( np.sum( quat ** 2 ) )

            # check the length is 1.0
            self.assertTrue(
                np.array_equal( result.toVector(), expected ),
                "Normalise quaternion incorrect"
                )
        non_identity()

    def test_length( self ):
        def identity():
            quat = utmath.Quaternion()
            result = utmath.abs(quat)

            expected = 1.0

            self.assertEqual(
                result,
                expected,
                "Identity quaternion length calculation incorrect"
                )
        identity()

        def non_identity():
            quat = np.asarray([1.0, 2.0, 3.0, 4.0])
            result = utmath.abs(utmath.Quaternion.fromVector(quat))

            expected = math.sqrt( np.sum( quat ** 2 ) )
            self.assertEqual(
                result,
                expected,
                "Quaternion length calculation incorrect"
                )
        non_identity()


    def test_apply_to_vector( self ):
        # Euler angle constructor uses X-Z-Y ordering
        
        def identity():
            quat = utmath.Quaternion()
            vec = utmath.Vector3d(1.0, 0.0, 0.0)

            result = quat * vec

            expected = vec

            self.assertTrue(
                np.array_equal( result, expected ),
                "Quaternion apply_to_vector incorrect with identity"
                )
        identity()

        def rotated_x():
            quat = utmath.Quaternion(  0.0, 0.0, math.pi )
            vec = utmath.Vector3d(0.0, 1.0, 0.0)

            result = quat * vec

            expected = -np.asarray(vec)

            self.assertTrue(
                np.allclose( result, expected ),
                "Quaternion apply_to_vector incorrect with rotation about X"
                )
        rotated_x()

        def rotated_y():
            quat = utmath.Quaternion( 0.0, math.pi, 0.0 )
            vec = utmath.Vector3d(1.0, 0.0, 0.0)

            result = quat * vec

            expected = -np.asarray(vec)

            self.assertTrue(
                np.allclose( result, expected ),
                "Quaternion apply_to_vector incorrect with rotation about Y"
                )
        rotated_y()

        def rotated_z():
            quat = utmath.Quaternion( math.pi, 0.0, 0.0 )
            vec = utmath.Vector3d(1.0, 0.0, 0.0)

            result = quat * vec

            expected = -np.asarray(vec)

            self.assertTrue(
                np.allclose( result, expected ),
                "Quaternion apply_to_vector incorrect with rotation about Y"
                )
        rotated_z()


# class test_quaternion(unittest.TestCase):
#     # many of these values are taken from searches on wolfram alpha
#     # code copied from pyrr unittests

#     def test_create(self):
#         result = math.Quaternion().toVector()
#         np.testing.assert_almost_equal(result, [0.,0.,0.,1.], decimal=5)
#         self.assertTrue(result.dtype == np.double)

#     def test_create_from_x_rotation(self):
#         result = quaternion.create_from_x_rotation(np.pi)
#         np.testing.assert_almost_equal(result, [1.,0.,0.,0.], decimal=3)
#         self.assertTrue(result.dtype == np.float)

#     def test_create_from_y_rotation(self):
#         result = quaternion.create_from_y_rotation(np.pi)
#         np.testing.assert_almost_equal(result, [0.,1.,0.,0.], decimal=3)
#         self.assertTrue(result.dtype == np.float)

#     def test_create_from_z_rotation(self):
#         result = quaternion.create_from_z_rotation(np.pi)
#         np.testing.assert_almost_equal(result, [0.,0.,1.,0.], decimal=3)
#         self.assertTrue(result.dtype == np.float)

#     def test_create_from_axis_rotation(self):
#         # wolfram alpha can be awesome sometimes
#         result = quaternion.create_from_axis_rotation([0.57735, 0.57735, 0.57735],np.pi)
#         np.testing.assert_almost_equal(result, [5.77350000e-01, 5.77350000e-01, 5.77350000e-01, 6.12323400e-17], decimal=3)
#         self.assertTrue(result.dtype == np.float)

#     def test_create_from_axis_rotation_non_normalised(self):
#         result = quaternion.create_from_axis_rotation([1.,1.,1.], np.pi)
#         np.testing.assert_almost_equal(result, [5.77350000e-01, 5.77350000e-01, 5.77350000e-01, 6.12323400e-17], decimal=3)
#         self.assertTrue(result.dtype == np.float)

#     def test_create_from_matrix_unit(self):
#         result = quaternion.create_from_matrix(np.eye(3))
#         np.testing.assert_almost_equal(result, [0.,0.,0.,1.], decimal=5)
#         self.assertTrue(result.dtype == np.float)

#     def test_create_from_matrix_x(self):
#         result = quaternion.create_from_matrix([
#             [1.,0.,0.],
#             [0.,-1.,0.],
#             [0.,0.,-1.],
#         ])
#         np.testing.assert_almost_equal(result, [1.,0.,0.,0.], decimal=5)
#         self.assertTrue(result.dtype == np.float)

#     def test_create_from_matrix_y(self):
#         result = quaternion.create_from_matrix([
#             [-1.,0.,0.],
#             [0.,1.,0.],
#             [0.,0.,-1.],
#         ])
#         np.testing.assert_almost_equal(result, [0.,1.,0.,0.], decimal=5)
#         self.assertTrue(result.dtype == np.float)

#     def test_create_from_matrix_z(self):
#         result = quaternion.create_from_matrix([
#             [-1.,0.,0.],
#             [0.,-1.,0.],
#             [0.,0.,1.],
#         ])
#         np.testing.assert_almost_equal(result, [0.,0.,1.,0.], decimal=5)
#         self.assertTrue(result.dtype == np.float)

#     @unittest.skip('Not implemented')
#     def test_create_from_eulers(self):
#         pass

#     @unittest.skip('Not implemented')
#     def test_create_from_inverse_of_eulers(self):
#         pass

#     def test_cross(self):
#         q1 = quaternion.create_from_x_rotation(np.pi / 2.0)
#         q2 = quaternion.create_from_x_rotation(-np.pi / 2.0)
#         result = quaternion.cross(q1, q2)
#         np.testing.assert_almost_equal(result, quaternion.create(), decimal=5)

#     def test_is_zero_length(self):
#         result = quaternion.is_zero_length([1.,0.,0.,0.])
#         self.assertFalse(result)

#     def test_is_zero_length_zero(self):
#         result = quaternion.is_zero_length([0.,0.,0.,0.])
#         self.assertTrue(result)

#     def test_is_non_zero_length(self):
#         result = quaternion.is_non_zero_length([1.,0.,0.,0.])
#         self.assertTrue(result)

#     def test_is_non_zero_length_zero(self):
#         result = quaternion.is_non_zero_length([0.,0.,0.,0.])
#         self.assertFalse(result)

#     def test_squared_length_identity(self):
#         result = quaternion.squared_length([0.,0.,0.,1.])
#         np.testing.assert_almost_equal(result, 1., decimal=5)

#     def test_squared_length(self):
#         result = quaternion.squared_length([1.,1.,1.,1.])
#         np.testing.assert_almost_equal(result, 4., decimal=5)

#     def test_squared_length_batch(self):
#         result = quaternion.squared_length([
#             [0.,0.,0.,1.],
#             [1.,1.,1.,1.],
#         ])
#         np.testing.assert_almost_equal(result, [1.,4.], decimal=5)

#     def test_length_identity(self):
#         result = quaternion.length([0.,0.,0.,1.])
#         np.testing.assert_almost_equal(result, 1., decimal=5)

#     def test_length(self):
#         result = quaternion.length([1.,1.,1.,1.])
#         np.testing.assert_almost_equal(result, 2., decimal=5)

#     def test_length_batch(self):
#         result = quaternion.length([
#             [0.,0.,0.,1.],
#             [1.,1.,1.,1.],
#         ])
#         np.testing.assert_almost_equal(result, [1.,2.], decimal=5)

#     def test_normalise_identity(self):
#         # normalise an identity quaternion
#         result = quaternion.normalise([0.,0.,0.,1.])
#         np.testing.assert_almost_equal(result, [0.,0.,0.,1.], decimal=5)

#     def test_normalise_non_identity(self):
#         # normalise an identity quaternion
#         result = quaternion.normalise([1.,2.,3.,4.])
#         np.testing.assert_almost_equal(result, [1./np.sqrt(30.),np.sqrt(2./15.),np.sqrt(3./10.),2.*np.sqrt(2./15.)], decimal=5)

#     def test_normalise_batch(self):
#         # normalise an identity quaternion
#         result = quaternion.normalise([
#             [0.,0.,0.,1.],
#             [1.,2.,3.,4.],
#         ])
#         expected = [
#             [0.,0.,0.,1.],
#             [1./np.sqrt(30.),np.sqrt(2./15.),np.sqrt(3./10.),2.*np.sqrt(2./15.)],
#         ]
#         np.testing.assert_almost_equal(result, expected, decimal=5)

#     def test_rotation_angle(self):
#         result = quaternion.rotation_angle([5.77350000e-01, 5.77350000e-01, 5.77350000e-01, 6.12323400e-17])
#         np.testing.assert_almost_equal(result, np.pi, decimal=5)

#     def test_rotation_axis(self):
#         result = quaternion.rotation_axis([5.77350000e-01, 5.77350000e-01, 5.77350000e-01, 6.12323400e-17])
#         np.testing.assert_almost_equal(result, [0.57735, 0.57735, 0.57735], decimal=5)

#     def test_dot_adjacent(self):
#         result = quaternion.dot([1.,0.,0.,0.], [0.,1.,0.,0.])
#         np.testing.assert_almost_equal(result, 0.0, decimal=5)

#     def test_dot_parallel(self):
#         result = quaternion.dot([0.,1.,0.,0.], [0.,1.,0.,0.])
#         np.testing.assert_almost_equal(result, 1.0, decimal=5)

#     def test_dot_angle(self):
#         result = quaternion.dot([.2,.2,0.,0.], [2.,-.2,0.,0.])
#         np.testing.assert_almost_equal(result, 0.36, decimal=5)

#     def test_dot_batch(self):
#         result = quaternion.dot([
#             [1.,0.,0.,0.],
#             [0.,1.,0.,0.],
#             [.2,.2,0.,0.]
#         ],[
#             [0.,1.,0.,0.],
#             [0.,1.,0.,0.],
#             [2.,-.2,0.,0.]
#         ])
#         expected = [0.,1.,0.36]
#         np.testing.assert_almost_equal(result, expected, decimal=5)

#     def test_conjugate(self):
#         #result = quaternion.conjugate([5.77350000e-01, 5.77350000e-01, 5.77350000e-01, 6.12323400e-17])
#         result = quaternion.conjugate([0.,0.,0.,1.])
#         np.testing.assert_almost_equal(result, [0.,0.,0.,1.], decimal=5)

#     def test_conjugate_rotation(self):
#         result = quaternion.conjugate([5.77350000e-01, 5.77350000e-01, 5.77350000e-01, 6.12323400e-17])
#         np.testing.assert_almost_equal(result, [-0.57735, -0.57735, -0.57735, 6.12323e-17], decimal=5)

#     @unittest.skip('Not implemented')
#     def test_power(self):
#         pass

#     def test_inverse(self):
#         result = quaternion.inverse([0.,0.,0.,1.])
#         np.testing.assert_almost_equal(result, [0.,0.,0.,1.], decimal=5)

#     def test_inverse_rotation(self):
#         result = quaternion.inverse([5.77350000e-01, 5.77350000e-01, 5.77350000e-01, 6.12323400e-17])
#         np.testing.assert_almost_equal(result, [-0.577351, -0.577351, -0.577351, 6.12324e-17], decimal=5)

#     def test_inverse_non_unit(self):
#         q = [1,2,3,4]
#         result = quaternion.inverse(q)
#         expected = quaternion.conjugate(q) / quaternion.length(q)
#         np.testing.assert_almost_equal(result, expected, decimal=5)

#     def test_negate_unit(self):
#         result = quaternion.negate([0.,0.,0.,1.])
#         np.testing.assert_almost_equal(result, [0.,0.,0.,-1.], decimal=5)

#     def test_negate(self):
#         result = quaternion.negate([1.,2.,3.,4.])
#         np.testing.assert_almost_equal(result, [-1.,-2.,-3.,-4.], decimal=5)

#     def test_apply_to_vector_unit_x(self):
#         result = quaternion.apply_to_vector([0.,0.,0.,1.],[1.,0.,0.])
#         np.testing.assert_almost_equal(result, [1.,0.,0.], decimal=5)

#     def test_apply_to_vector_x(self):
#         quat = quaternion.create_from_x_rotation(np.pi / 2.)
#         self.assertTrue(np.allclose(quaternion.apply_to_vector(quat,[1.,0.,0.]), [1.,0.,0.]))
#         self.assertTrue(np.allclose(quaternion.apply_to_vector(quat,[0.,1.,0.]), [0.,0.,-1.]))
#         self.assertTrue(np.allclose(quaternion.apply_to_vector(quat,[0.,0.,1.]), [0.,1.,0.]))

#     def test_apply_to_vector_y(self):
#         quat = quaternion.create_from_y_rotation(np.pi / 2.)
#         self.assertTrue(np.allclose(quaternion.apply_to_vector(quat,[1.,0.,0.]), [0.,0.,1.]))
#         self.assertTrue(np.allclose(quaternion.apply_to_vector(quat,[0.,1.,0.]), [0.,1.,0.]))
#         self.assertTrue(np.allclose(quaternion.apply_to_vector(quat,[0.,0.,1.]), [-1.,0.,0.]))

#     def test_apply_to_vector_z(self):
#         quat = quaternion.create_from_z_rotation(np.pi / 2.)
#         self.assertTrue(np.allclose(quaternion.apply_to_vector(quat,[1.,0.,0.]), [0.,-1.,0.]))
#         self.assertTrue(np.allclose(quaternion.apply_to_vector(quat,[0.,1.,0.]), [1.,0.,0.]))
#         self.assertTrue(np.allclose(quaternion.apply_to_vector(quat,[0.,0.,1.]), [0.,0.,1.]))

if __name__ == '__main__':
    unittest.main()