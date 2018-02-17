#include <boost/python.hpp>
#include <boost/python/implicit.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/to_python_converter.hpp>

#include <boost/python/object.hpp>
#include <boost/python/str.hpp>
#include <boost/python/extract.hpp>

#include <boost/optional.hpp>
#include <boost/utility/typed_in_place_factory.hpp>

#include <boost/filesystem.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp>

#include <istream>
#include <streambuf>
#include <iostream>

#include <utMeasurement/Measurement.h>

#include <utUtil/Logging.h>
#include <utUtil/Exception.h>
#include <utUtil/CalibFile.h>

using namespace Ubitrack;

namespace bp = boost::python;

namespace {


template< class T >
const std::string serializeMeasurement(Measurement::Measurement<T>& m) {
	std::ostringstream stream;
	boost::archive::text_oarchive packet( stream );
	packet << m;

	// Should be like this ..
	//now you wrap that as buffer
	//PyObject* py_buf = PyBuffer_FromReadMemory(stream.str().data(), stream.str().size());
	//bp::object retval = bp::object(bp::handle<>(py_buf));
	//return retval;
	return stream.str();
}

template< class T >
const std::string serializeMeasurementNetwork(Measurement::Timestamp sendtime, bp::object name_obj, Measurement::Measurement<T>& m) {
	bp::extract<std::string> name_obj_ext(name_obj);
	if (!name_obj_ext.check()) {
		throw std::runtime_error("2nd argument must be python string");
	}
	std::string name = name_obj_ext();


	std::ostringstream stream;
	boost::archive::text_oarchive packet( stream );
	std::string suffix("\n");

	// serialize the Measurement::Measurement, component name and current local time
	packet << name;
	packet << m;
	packet << sendtime;
	packet << suffix;

	//Should be like this: 
	// now you wrap that as buffer
	//PyObject* py_buf = PyBuffer_FromReadMemory(stream.str().data(), stream.str().size());
	//bp::object retval = bp::object(bp::handle<>(py_buf));
	//return retval;
	return stream.str();
}

template< class T >
Measurement::Measurement<T> deserializeMeasurement(bp::object obj) {
	bp::extract<std::string> obj_ext(obj);
	if (!obj_ext.check()) {
		throw std::runtime_error("argument must be python string");
	}
	std::string data = obj_ext();

	Measurement::Measurement<T> result ( 0, boost::shared_ptr< T >( new T() ) );

	// create ifstream
	std::istringstream stream( data );

	// create iarchive
	boost::archive::text_iarchive archive( stream );

	try {
	    // read data
	    archive >> result;
    } catch (std::exception& ) {
        UBITRACK_THROW( "Wrong file format" );
    }
	return result;
}

template< class T >
bp::object deserializeMeasurementNetwork(bp::object obj) {
	bp::extract<std::string> obj_ext(obj);
	if (!obj_ext.check()) {
		throw std::runtime_error("argument must be python string");
	}
	std::string data = obj_ext();

    std::istringstream stream( data );
    boost::archive::text_iarchive archive( stream );

    std::string name;
	Measurement::Measurement<T> mm ( 0, boost::shared_ptr< T >( new T() ) );
    Measurement::Timestamp sendtime;

	try {
	    // read data
	    archive >> name;
        archive >> mm;
        archive >> sendtime;
    } catch (std::exception& ) {
        UBITRACK_THROW( "Wrong file format" );
    }

	// create list as result
	bp::list result;
	result.append(bp::object(sendtime));
	result.append(bp::object(name));
	result.append(bp::object(mm));
	return result;
}

// not nice, but we don't know which type the archive has until we know its name ...
std::string nameFromNetworkArchive(bp::object obj) {
	bp::extract<std::string> obj_ext(obj);
	if (!obj_ext.check()) {
		throw std::runtime_error("argument must be python string");
	}
	std::string data = obj_ext();

	std::istringstream stream( data );
    boost::archive::text_iarchive archive( stream );

    std::string name;
	try {
	    // read data
	    archive >> name;
    } catch (std::exception& ) {
        UBITRACK_THROW( "Wrong file format" );
    }

	return name;
}


}

BOOST_PYTHON_MODULE(_serialization)
{

    bp::def("serializeScalar", &serializeMeasurement< Math::Scalar< double > >);
    bp::def("serializePose", &serializeMeasurement< Math::Pose >);
    bp::def("serializeErrorPose", &serializeMeasurement< Math::ErrorPose >);
    bp::def("serializeErrorPosition", &serializeMeasurement< Math::ErrorVector< double, 3 > >);
    bp::def("serializePosition", &serializeMeasurement< Math::Vector3d >);
    bp::def("serializeRotation", &serializeMeasurement< Math::Quaternion >);
    bp::def("serializeMatrix3x3", &serializeMeasurement< Math::Matrix3x3d >);
    bp::def("serializeMatrix3x4", &serializeMeasurement< Math::Matrix3x4d >);
    bp::def("serializeMatrix4x4", &serializeMeasurement< Math::Matrix4x4d >);
    bp::def("serializeVector4D", &serializeMeasurement< Math::Vector4d >);
    bp::def("serializeVector8D", &serializeMeasurement< Math::Vector< double, 8 > >);
    bp::def("serializePositionList", &serializeMeasurement< std::vector< Math::Vector3d > >);
    bp::def("serializeCameraIntrinsics", &serializeMeasurement< Math::CameraIntrinsics<double> >);
    bp::def("serializePositionList2", &serializeMeasurement< std::vector< Math::Vector2d > >);
    bp::def("serializePoseList", &serializeMeasurement< std::vector< Math::Pose > >);
    bp::def("serializeDistanceList", &serializeMeasurement< std::vector< Math::Scalar< double > > >);

    bp::def("serializeNetworkScalar", &serializeMeasurementNetwork< Math::Scalar< double > >);
    bp::def("serializeNetworkPose", &serializeMeasurementNetwork< Math::Pose >);
    bp::def("serializeNetworkErrorPose", &serializeMeasurementNetwork< Math::ErrorPose >);
    bp::def("serializeNetworkErrorPosition", &serializeMeasurementNetwork< Math::ErrorVector< double, 3 > >);
    bp::def("serializeNetworkPosition", &serializeMeasurementNetwork< Math::Vector3d >);
    bp::def("serializeNetworkRotation", &serializeMeasurementNetwork< Math::Quaternion >);
    bp::def("serializeNetworkMatrix3x3", &serializeMeasurementNetwork< Math::Matrix3x3d >);
    bp::def("serializeNetworkMatrix3x4", &serializeMeasurementNetwork< Math::Matrix3x4d >);
    bp::def("serializeNetworkMatrix4x4", &serializeMeasurementNetwork< Math::Matrix4x4d >);
    bp::def("serializeNetworkVector4D", &serializeMeasurementNetwork< Math::Vector4d >);
    bp::def("serializeNetworkVector8D", &serializeMeasurementNetwork< Math::Vector< double, 8 > >);
    bp::def("serializeNetworkCameraIntrinsics", &serializeMeasurementNetwork< Math::CameraIntrinsics<double> >);
    bp::def("serializeNetworkPositionList", &serializeMeasurementNetwork< std::vector< Math::Vector3d > >);
    bp::def("serializeNetworkPositionList2", &serializeMeasurementNetwork< std::vector< Math::Vector2d > >);
    bp::def("serializeNetworkPoseList", &serializeMeasurementNetwork< std::vector< Math::Pose > >);
    bp::def("serializeNetworkDistanceList", &serializeMeasurementNetwork< std::vector< Math::Scalar< double > > >);

    bp::def("deserializeScalar", &deserializeMeasurement< Math::Scalar< double > >);
    bp::def("deserializePose", &deserializeMeasurement< Math::Pose >);
    bp::def("deserializeErrorPose", &deserializeMeasurement< Math::ErrorPose >);
    bp::def("deserializeErrorPosition", &deserializeMeasurement< Math::ErrorVector< double, 3 > >);
    bp::def("deserializePosition", &deserializeMeasurement< Math::Vector3d >);
    bp::def("deserializeRotation", &deserializeMeasurement< Math::Quaternion >);
    bp::def("deserializeMatrix3x3", &deserializeMeasurement< Math::Matrix3x3d >);
    bp::def("deserializeMatrix3x4", &deserializeMeasurement< Math::Matrix3x4d >);
    bp::def("deserializeMatrix4x4", &deserializeMeasurement< Math::Matrix4x4d >);
    bp::def("deserializeVector4D", &deserializeMeasurement< Math::Vector4d >);
    bp::def("deserializeVector8D", &deserializeMeasurement< Math::Vector< double, 8 > >);
    bp::def("deserializeCameraIntrinsics", &deserializeMeasurement< Math::CameraIntrinsics<double> >);
    bp::def("deserializePositionList", &deserializeMeasurement< std::vector< Math::Vector3d > >);
    bp::def("deserializePositionList2", &deserializeMeasurement< std::vector< Math::Vector2d > >);
    bp::def("deserializePoseList", &deserializeMeasurement< std::vector< Math::Pose > >);
    bp::def("deserializeDistanceList", &deserializeMeasurement< std::vector< Math::Scalar< double > > >);

    bp::def("deserializeNetworkScalar", &deserializeMeasurementNetwork< Math::Scalar< double > >);
    bp::def("deserializeNetworkPose", &deserializeMeasurementNetwork< Math::Pose >);
    bp::def("deserializeNetworkErrorPose", &deserializeMeasurementNetwork< Math::ErrorPose >);
    bp::def("deserializeNetworkErrorPosition", &deserializeMeasurementNetwork< Math::ErrorVector< double, 3 > >);
    bp::def("deserializeNetworkPosition", &deserializeMeasurementNetwork< Math::Vector3d >);
    bp::def("deserializeNetworkRotation", &deserializeMeasurementNetwork< Math::Quaternion >);
    bp::def("deserializeNetworkMatrix3x3", &deserializeMeasurementNetwork< Math::Matrix3x3d >);
    bp::def("deserializeNetworkMatrix3x4", &deserializeMeasurementNetwork< Math::Matrix3x4d >);
    bp::def("deserializeNetworkMatrix4x4", &deserializeMeasurementNetwork< Math::Matrix4x4d >);
    bp::def("deserializeNetworkVector4D", &deserializeMeasurementNetwork< Math::Vector4d >);
    bp::def("deserializeNetworkVector8D", &deserializeMeasurementNetwork< Math::Vector< double, 8 > >);
    bp::def("deserializeNetworkCameraIntrinsics", &deserializeMeasurementNetwork< Math::CameraIntrinsics<double> >);
    bp::def("deserializeNetworkPositionList", &deserializeMeasurementNetwork< std::vector< Math::Vector3d > >);
    bp::def("deserializeNetworkPositionList2", &deserializeMeasurementNetwork< std::vector< Math::Vector2d > >);
    bp::def("deserializeNetworkPoseList", &deserializeMeasurementNetwork< std::vector< Math::Pose > >);
    bp::def("deserializeNetworkDistanceList", &deserializeMeasurementNetwork< std::vector< Math::Scalar< double > > >);

	bp::def("nameFromNetworkArchive", &nameFromNetworkArchive);
}
