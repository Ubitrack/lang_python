#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#ifndef HAVE_BOOST_NUMPY
#include <boost/python/numpy.hpp>
#else
#include <boost/numpy.hpp>
#endif

#include <numpy/arrayobject.h>
#include <complex>

#include <boost/python.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/to_python_converter.hpp>

#include <utHaptics/Function/PhantomForwardKinematics.h>
#include <utHaptics/PhantomLMCalibration.h>
#include <utHaptics/PhantomLMGimbalCalibration.h>

using namespace Ubitrack;

namespace bp = boost::python;
#ifndef HAVE_BOOST_NUMPY
namespace bn = boost::python::numpy;
#else
namespace bn = boost::numpy;
#endif

namespace {

    Math::Pose computePhantomForwardKinematicsPose(
            const bn::ndarray &py_joint_angles,
            const bn::ndarray &py_gimbal_angles,
            const bn::ndarray &py_ja_correction,
            const bn::ndarray &py_ga_correction,
            const bn::ndarray &py_joint_lengths,
            const bn::ndarray &py_origin_calib) {

        if (py_joint_angles.get_dtype() != bn::dtype::get_builtin<double>()) {
            PyErr_SetString(PyExc_TypeError, "Incorrect array data type for joint_angles");
            bp::throw_error_already_set();
        }

        if (py_gimbal_angles.get_dtype() != bn::dtype::get_builtin<double>()) {
            PyErr_SetString(PyExc_TypeError, "Incorrect array data type for gimbal_angles");
            bp::throw_error_already_set();
        }

        if (py_ja_correction.get_dtype() != bn::dtype::get_builtin<double>()) {
            PyErr_SetString(PyExc_TypeError, "Incorrect array data type for joint_angle correction matrix");
            bp::throw_error_already_set();
        }

        if (py_ga_correction.get_dtype() != bn::dtype::get_builtin<double>()) {
            PyErr_SetString(PyExc_TypeError, "Incorrect array data type for gimbal_angle correction matrix");
            bp::throw_error_already_set();
        }

        if (py_joint_lengths.get_dtype() != bn::dtype::get_builtin<double>()) {
            PyErr_SetString(PyExc_TypeError, "Incorrect array data type for joint_lengths");
            bp::throw_error_already_set();
        }

        if (py_origin_calib.get_dtype() != bn::dtype::get_builtin<double>()) {
            PyErr_SetString(PyExc_TypeError, "Incorrect array data type for origin_calib");
            bp::throw_error_already_set();
        }


        if ((py_joint_angles.get_nd() != 1) || (py_joint_angles.shape(0) != 3)) {
            PyErr_SetString(PyExc_TypeError, "Incorrect dimensions for joint_angles");
            bp::throw_error_already_set();
        }

        if ((py_gimbal_angles.get_nd() != 1) || (py_gimbal_angles.shape(0) != 3)) {
            PyErr_SetString(PyExc_TypeError, "Incorrect dimensions for gimbal_angles");
            bp::throw_error_already_set();
        }

        if ((py_ja_correction.get_nd() != 2) || (py_ja_correction.shape(0) != 3)|| (py_ja_correction.shape(1) != 3)) {
            PyErr_SetString(PyExc_TypeError, "Incorrect dimensions for joint_angle correction matrix");
            bp::throw_error_already_set();
        }

        if ((py_ga_correction.get_nd() != 2) || (py_ga_correction.shape(0) != 3)|| (py_ga_correction.shape(1) != 3)) {
            PyErr_SetString(PyExc_TypeError, "Incorrect dimensions for gimbal_angle correction matrix");
            bp::throw_error_already_set();
        }

        if ((py_joint_lengths.get_nd() != 1) || (py_joint_lengths.shape(0) != 2)) {
            PyErr_SetString(PyExc_TypeError, "Incorrect dimensions for joint_lengths");
            bp::throw_error_already_set();
        }

        if ((py_origin_calib.get_nd() != 1) || (py_origin_calib.shape(0) != 3)) {
            PyErr_SetString(PyExc_TypeError, "Incorrect dimensions for origin_calib");
            bp::throw_error_already_set();
        }

        Math::Vector< double, 3 > joint_angles(reinterpret_cast<double*>(py_joint_angles.get_data()));
        Math::Vector< double, 3 > gimbal_angles(reinterpret_cast<double*>(py_gimbal_angles.get_data()));
        Math::Matrix< double, 3, 3 > ja_correction(reinterpret_cast<double*>(py_ja_correction.get_data()));
        Math::Matrix< double, 3, 3 > ga_correction(reinterpret_cast<double*>(py_ga_correction.get_data()));
        Math::Vector< double, 2 > joint_lengths(reinterpret_cast<double*>(py_joint_lengths.get_data()));
        Math::Vector< double, 3 > origin_calib(reinterpret_cast<double*>(py_origin_calib.get_data()));

        return Haptics::Function::computePhantomForwardKinematicsPose(joint_angles, gimbal_angles, ja_correction, ga_correction, joint_lengths, origin_calib);
    }

    Math::Matrix< double, 3, 3 > computePhantomLMCalibration(
            const std::vector< Ubitrack::Math::Vector< double, 3> > &joint_angles,
            const std::vector< Ubitrack::Math::Vector< double, 3> > &points,
            const bn::ndarray &py_joint_lengths,
            const bn::ndarray &py_origin_calib,
            const float optimizationStepSize,
            const float optimizationStepFactor ) {

        if (py_joint_lengths.get_dtype() != bn::dtype::get_builtin<double>()) {
            PyErr_SetString(PyExc_TypeError, "Incorrect array data type for joint_lengths");
            bp::throw_error_already_set();
        }

        if (py_origin_calib.get_dtype() != bn::dtype::get_builtin<double>()) {
            PyErr_SetString(PyExc_TypeError, "Incorrect array data type for origin_calib");
            bp::throw_error_already_set();
        }


        if ((py_joint_lengths.get_nd() != 1) || (py_joint_lengths.shape(0) != 2)) {
            PyErr_SetString(PyExc_TypeError, "Incorrect dimensions for joint_lengths");
            bp::throw_error_already_set();
        }

        if ((py_origin_calib.get_nd() != 1) || (py_origin_calib.shape(0) != 3)) {
            PyErr_SetString(PyExc_TypeError, "Incorrect dimensions for origin_calib");
            bp::throw_error_already_set();
        }


        Math::Vector< double, 2 > joint_lengths(reinterpret_cast<double*>(py_joint_lengths.get_data()));
        Math::Vector< double, 3 > origin_calib(reinterpret_cast<double*>(py_origin_calib.get_data()));

        return Haptics::computePhantomLMCalibration(joint_angles, points, joint_lengths( 0 ), joint_lengths( 1 ), origin_calib, (double)optimizationStepSize, (double)optimizationStepFactor);
    }

    Math::Matrix< double, 3, 3 > computePhantomLMGimbalCalibration(
            const std::vector< Ubitrack::Math::Vector< double, 3> > &joint_angles,
            const std::vector< Ubitrack::Math::Vector< double, 3> > &gimbal_angles,
            const std::vector< Ubitrack::Math::Vector< double, 3> > &zref,
            const bn::ndarray &py_ja_correction,
            const bn::ndarray &py_joint_lengths,
            const bn::ndarray &py_origin_calib,
            const float optimizationStepSize,
            const float optimizationStepFactor) {

        if (py_ja_correction.get_dtype() != bn::dtype::get_builtin<double>()) {
            PyErr_SetString(PyExc_TypeError, "Incorrect array data type for joint_angle correction matrix");
            bp::throw_error_already_set();
        }

        if (py_joint_lengths.get_dtype() != bn::dtype::get_builtin<double>()) {
            PyErr_SetString(PyExc_TypeError, "Incorrect array data type for joint_lengths");
            bp::throw_error_already_set();
        }

        if (py_origin_calib.get_dtype() != bn::dtype::get_builtin<double>()) {
            PyErr_SetString(PyExc_TypeError, "Incorrect array data type for origin_calib");
            bp::throw_error_already_set();
        }

        if ((py_ja_correction.get_nd() != 2) || (py_ja_correction.shape(0) != 3)|| (py_ja_correction.shape(1) != 3)) {
            PyErr_SetString(PyExc_TypeError, "Incorrect dimensions for joint_angle correction matrix");
            bp::throw_error_already_set();
        }

        if ((py_joint_lengths.get_nd() != 1) || (py_joint_lengths.shape(0) != 2)) {
            PyErr_SetString(PyExc_TypeError, "Incorrect dimensions for joint_lengths");
            bp::throw_error_already_set();
        }

        if ((py_origin_calib.get_nd() != 1) || (py_origin_calib.shape(0) != 3)) {
            PyErr_SetString(PyExc_TypeError, "Incorrect dimensions for origin_calib");
            bp::throw_error_already_set();
        }

        Math::Matrix< double, 3, 3 > ja_correction(reinterpret_cast<double*>(py_ja_correction.get_data()));
        Math::Vector< double, 2 > joint_lengths(reinterpret_cast<double*>(py_joint_lengths.get_data()));
        Math::Vector< double, 3 > origin_calib(reinterpret_cast<double*>(py_origin_calib.get_data()));

        return Haptics::computePhantomLMGimbalCalibration(joint_angles, gimbal_angles, zref, joint_lengths( 0 ), joint_lengths( 1 ), ja_correction, origin_calib, (double)optimizationStepSize, (double)optimizationStepFactor);
    }

}

BOOST_PYTHON_MODULE(_uthaptics)
{
    bp::def("computePhantomLMCalibration", &computePhantomLMCalibration);
    bp::def("computePhantomLMGimbalCalibration", &computePhantomLMGimbalCalibration);
    bp::def("computePhantomForwardKinematicsPose", &computePhantomForwardKinematicsPose);
}
