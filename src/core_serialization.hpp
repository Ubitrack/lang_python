/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the 
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */


/**
 * @ingroup python
 * @file
 * Ubitrack Python Bindings - Ubitrack::Math Module.
 * @author Ulrich Eck <ulrich.eck@tum.de>
 */

#include "ubitrack_python/opaque_types.h"
#include "ubitrack_python/pyubitrack.h"

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

template< class T >
py::bytes serializeMeasurement(Ubitrack::Measurement::Measurement<T>& m) {
	std::ostringstream stream;
	boost::archive::text_oarchive packet( stream );
	packet << m;
	return py::bytes(stream.str());
}

template< class T >
py::bytes serializeMeasurementNetwork(
	Ubitrack::Measurement::Timestamp sendtime, 
	const std::string& name, 
	Ubitrack::Measurement::Measurement<T>& m) {

	std::ostringstream stream;
	boost::archive::text_oarchive packet( stream );
	std::string suffix("\n");

	// serialize the Measurement::Measurement, component name and current local time
	packet << name;
	packet << m;
	packet << sendtime;
	packet << suffix;
	return py::bytes(stream.str());
}

template< class T >
Ubitrack::Measurement::Measurement<T> deserializeMeasurement(const std::string& data) {

	Ubitrack::Measurement::Measurement<T> result ( 0, boost::shared_ptr< T >( new T() ) );

	// create ifstream
	std::istringstream stream( data );

	// create iarchive
	boost::archive::text_iarchive archive( stream );

	try {
	    // read data
	    archive >> result;
    } catch (std::exception& ) {
        throw std::runtime_error( "Wrong file format" );
    }
	return result;
}

template< class T >
std::tuple<Ubitrack::Measurement::Timestamp, std::string, Ubitrack::Measurement::Measurement<T>> 
deserializeMeasurementNetwork(const std::string& data) {

    std::istringstream stream( data );
    boost::archive::text_iarchive archive( stream );

    std::string name;
	Ubitrack::Measurement::Measurement<T> mm ( 0, boost::shared_ptr< T >( new T() ) );
    Ubitrack::Measurement::Timestamp sendtime;

	try {
	    // read data
	    archive >> name;
        archive >> mm;
        archive >> sendtime;
    } catch (std::exception& ) {
        throw std::runtime_error( "Wrong file format" );
    }

    return std::make_tuple(sendtime, name, mm);
}

// not nice, but we don't know which type the archive has until we know its name ...
py::bytes nameFromNetworkArchive(const std::string& data) {
	std::istringstream stream( data );
    boost::archive::text_iarchive archive( stream );

    std::string name;
	try {
	    // read data
	    archive >> name;
    } catch (std::exception& ) {
        throw std::runtime_error( "Wrong file format" );
    }

	return py::bytes(name);
}


void bind_utSerialization(py::module& m)
{
    m.def("serializeScalar", &serializeMeasurement< Ubitrack::Math::Scalar< double > >);
    m.def("serializePose", &serializeMeasurement< Ubitrack::Math::Pose >);
    m.def("serializeErrorPose", &serializeMeasurement< Ubitrack::Math::ErrorPose >);
    m.def("serializeErrorPosition", &serializeMeasurement< Ubitrack::Math::ErrorVector< double, 3 > >);
    m.def("serializePosition", &serializeMeasurement< Ubitrack::Math::Vector3d >);
    m.def("serializeRotation", &serializeMeasurement< Ubitrack::Math::Quaternion >);
    m.def("serializeMatrix3x3", &serializeMeasurement< Ubitrack::Math::Matrix3x3d >);
    m.def("serializeMatrix3x4", &serializeMeasurement< Ubitrack::Math::Matrix3x4d >);
    m.def("serializeMatrix4x4", &serializeMeasurement< Ubitrack::Math::Matrix4x4d >);
    m.def("serializeVector4D", &serializeMeasurement< Ubitrack::Math::Vector4d >);
    m.def("serializeVector8D", &serializeMeasurement< Ubitrack::Math::Vector< double, 8 > >);
    m.def("serializePositionList", &serializeMeasurement< std::vector< Ubitrack::Math::Vector3d > >);
    m.def("serializeCameraIntrinsics", &serializeMeasurement< Ubitrack::Math::CameraIntrinsics<double> >);
    m.def("serializePositionList2", &serializeMeasurement< std::vector< Ubitrack::Math::Vector2d > >);
    m.def("serializePoseList", &serializeMeasurement< std::vector< Ubitrack::Math::Pose > >);
    m.def("serializeDistanceList", &serializeMeasurement< std::vector< Ubitrack::Math::Scalar< double > > >);

    m.def("serializeNetworkScalar", &serializeMeasurementNetwork< Ubitrack::Math::Scalar< double > >);
    m.def("serializeNetworkPose", &serializeMeasurementNetwork< Ubitrack::Math::Pose >);
    m.def("serializeNetworkErrorPose", &serializeMeasurementNetwork< Ubitrack::Math::ErrorPose >);
    m.def("serializeNetworkErrorPosition", &serializeMeasurementNetwork< Ubitrack::Math::ErrorVector< double, 3 > >);
    m.def("serializeNetworkPosition", &serializeMeasurementNetwork< Ubitrack::Math::Vector3d >);
    m.def("serializeNetworkRotation", &serializeMeasurementNetwork< Ubitrack::Math::Quaternion >);
    m.def("serializeNetworkMatrix3x3", &serializeMeasurementNetwork< Ubitrack::Math::Matrix3x3d >);
    m.def("serializeNetworkMatrix3x4", &serializeMeasurementNetwork< Ubitrack::Math::Matrix3x4d >);
    m.def("serializeNetworkMatrix4x4", &serializeMeasurementNetwork< Ubitrack::Math::Matrix4x4d >);
    m.def("serializeNetworkVector4D", &serializeMeasurementNetwork< Ubitrack::Math::Vector4d >);
    m.def("serializeNetworkVector8D", &serializeMeasurementNetwork< Ubitrack::Math::Vector< double, 8 > >);
    m.def("serializeNetworkCameraIntrinsics", &serializeMeasurementNetwork< Ubitrack::Math::CameraIntrinsics<double> >);
    m.def("serializeNetworkPositionList", &serializeMeasurementNetwork< std::vector< Ubitrack::Math::Vector3d > >);
    m.def("serializeNetworkPositionList2", &serializeMeasurementNetwork< std::vector< Ubitrack::Math::Vector2d > >);
    m.def("serializeNetworkPoseList", &serializeMeasurementNetwork< std::vector< Ubitrack::Math::Pose > >);
    m.def("serializeNetworkDistanceList", &serializeMeasurementNetwork< std::vector< Ubitrack::Math::Scalar< double > > >);

    m.def("deserializeScalar", &deserializeMeasurement< Ubitrack::Math::Scalar< double > >);
    m.def("deserializePose", &deserializeMeasurement< Ubitrack::Math::Pose >);
    m.def("deserializeErrorPose", &deserializeMeasurement< Ubitrack::Math::ErrorPose >);
    m.def("deserializeErrorPosition", &deserializeMeasurement< Ubitrack::Math::ErrorVector< double, 3 > >);
    m.def("deserializePosition", &deserializeMeasurement< Ubitrack::Math::Vector3d >);
    m.def("deserializeRotation", &deserializeMeasurement< Ubitrack::Math::Quaternion >);
    m.def("deserializeMatrix3x3", &deserializeMeasurement< Ubitrack::Math::Matrix3x3d >);
    m.def("deserializeMatrix3x4", &deserializeMeasurement< Ubitrack::Math::Matrix3x4d >);
    m.def("deserializeMatrix4x4", &deserializeMeasurement< Ubitrack::Math::Matrix4x4d >);
    m.def("deserializeVector4D", &deserializeMeasurement< Ubitrack::Math::Vector4d >);
    m.def("deserializeVector8D", &deserializeMeasurement< Ubitrack::Math::Vector< double, 8 > >);
    m.def("deserializeCameraIntrinsics", &deserializeMeasurement< Ubitrack::Math::CameraIntrinsics<double> >);
    m.def("deserializePositionList", &deserializeMeasurement< std::vector< Ubitrack::Math::Vector3d > >);
    m.def("deserializePositionList2", &deserializeMeasurement< std::vector< Ubitrack::Math::Vector2d > >);
    m.def("deserializePoseList", &deserializeMeasurement< std::vector< Ubitrack::Math::Pose > >);
    m.def("deserializeDistanceList", &deserializeMeasurement< std::vector< Ubitrack::Math::Scalar< double > > >);

    m.def("deserializeNetworkScalar", &deserializeMeasurementNetwork< Ubitrack::Math::Scalar< double > >);
    m.def("deserializeNetworkPose", &deserializeMeasurementNetwork< Ubitrack::Math::Pose >);
    m.def("deserializeNetworkErrorPose", &deserializeMeasurementNetwork< Ubitrack::Math::ErrorPose >);
    m.def("deserializeNetworkErrorPosition", &deserializeMeasurementNetwork< Ubitrack::Math::ErrorVector< double, 3 > >);
    m.def("deserializeNetworkPosition", &deserializeMeasurementNetwork< Ubitrack::Math::Vector3d >);
    m.def("deserializeNetworkRotation", &deserializeMeasurementNetwork< Ubitrack::Math::Quaternion >);
    m.def("deserializeNetworkMatrix3x3", &deserializeMeasurementNetwork< Ubitrack::Math::Matrix3x3d >);
    m.def("deserializeNetworkMatrix3x4", &deserializeMeasurementNetwork< Ubitrack::Math::Matrix3x4d >);
    m.def("deserializeNetworkMatrix4x4", &deserializeMeasurementNetwork< Ubitrack::Math::Matrix4x4d >);
    m.def("deserializeNetworkVector4D", &deserializeMeasurementNetwork< Ubitrack::Math::Vector4d >);
    m.def("deserializeNetworkVector8D", &deserializeMeasurementNetwork< Ubitrack::Math::Vector< double, 8 > >);
    m.def("deserializeNetworkCameraIntrinsics", &deserializeMeasurementNetwork< Ubitrack::Math::CameraIntrinsics<double> >);
    m.def("deserializeNetworkPositionList", &deserializeMeasurementNetwork< std::vector< Ubitrack::Math::Vector3d > >);
    m.def("deserializeNetworkPositionList2", &deserializeMeasurementNetwork< std::vector< Ubitrack::Math::Vector2d > >);
    m.def("deserializeNetworkPoseList", &deserializeMeasurementNetwork< std::vector< Ubitrack::Math::Pose > >);
    m.def("deserializeNetworkDistanceList", &deserializeMeasurementNetwork< std::vector< Ubitrack::Math::Scalar< double > > >);

	m.def("nameFromNetworkArchive", &nameFromNetworkArchive);

}