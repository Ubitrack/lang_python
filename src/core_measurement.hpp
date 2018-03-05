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
 * Ubitrack Python Bindings - Math Module.
 * @author Ulrich Eck <ulrich.eck@tum.de>
 */

#include "ubitrack_python/opaque_types.h"
#include "ubitrack_python/pyubitrack.h"

#include <boost/shared_ptr.hpp>

#include <utMeasurement/Measurement.h>

template<typename T>
void bind_measurement_type(py::module &m, const std::string &type_name, const std::string &doc_txt)
{
    py::class_<T>(m, type_name.c_str(), doc_txt.c_str())
    	.def(py::init<>())
		.def(py::init<Ubitrack::Measurement::Timestamp>())
		.def(py::init<Ubitrack::Measurement::Timestamp, const typename T:: value_type& >())
		.def("time", (Ubitrack::Measurement::Timestamp (T::*)() const)&T::time)
		.def("set_time", (void (T::*)(Ubitrack::Measurement::Timestamp))&T::time)
		.def("invalid", &T::invalid)
		.def("invalidate", &T::invalidate)
		.def("get", &T::get, py::return_value_policy::reference_internal)
		;
}


void bind_utMeasurement(py::module& m)
{
	m.def("now", Ubitrack::Measurement::now);

	bind_measurement_type<Ubitrack::Measurement::Distance>(m, "Distance", "Measurement::Distance");
	bind_measurement_type<Ubitrack::Measurement::Button>(m, "Button", "Measurement::Button");
	bind_measurement_type<Ubitrack::Measurement::Position2D>(m, "Position2D", "Measurement::Position2D");
	bind_measurement_type<Ubitrack::Measurement::Position>(m, "Position", "Measurement::Position");
	bind_measurement_type<Ubitrack::Measurement::Vector4D>(m, "Vector4D", "Measurement::Vector4D");
	bind_measurement_type<Ubitrack::Measurement::Vector8D>(m, "Vector8D", "Measurement::Vector8D");
	bind_measurement_type<Ubitrack::Measurement::Rotation>(m, "Rotation", "Measurement::Rotation");
	bind_measurement_type<Ubitrack::Measurement::Matrix3x3>(m, "Matrix3x3", "Measurement::Matrix3x3");
	bind_measurement_type<Ubitrack::Measurement::Matrix3x4>(m, "Matrix3x4", "Measurement::Matrix3x4");
	bind_measurement_type<Ubitrack::Measurement::Matrix4x4>(m, "Matrix4x4", "Measurement::Matrix4x4");
	bind_measurement_type<Ubitrack::Measurement::Pose>(m, "Pose", "Measurement::Pose");
	bind_measurement_type<Ubitrack::Measurement::ErrorPose>(m, "ErrorPose", "Measurement::ErrorPose");
	bind_measurement_type<Ubitrack::Measurement::ErrorPosition2>(m, "ErrorPosition2", "Measurement::ErrorPosition2");
	bind_measurement_type<Ubitrack::Measurement::ErrorPosition>(m, "ErrorPosition", "Measurement::ErrorPosition");
	bind_measurement_type<Ubitrack::Measurement::RotationVelocity>(m, "RotationVelocity", "Measurement::RotationVelocity");
	bind_measurement_type<Ubitrack::Measurement::CameraIntrinsics>(m, "CameraIntrinsics", "Measurement::CameraIntrinsics");

	bind_measurement_type<Ubitrack::Measurement::DistanceList>(m, "DistanceList", "Measurement::DistanceList");
	bind_measurement_type<Ubitrack::Measurement::IDList>(m, "IDList", "Measurement::IDList");
	bind_measurement_type<Ubitrack::Measurement::PositionList2>(m, "PositionList2", "Measurement::PositionList2");
	bind_measurement_type<Ubitrack::Measurement::PositionList>(m, "PositionList", "Measurement::PositionList");
	//Vector4D
	//Vector8D
	//Rotation
	//Matrix33
	//Matrix34
	//Matrix44
	bind_measurement_type<Ubitrack::Measurement::PoseList>(m, "PoseList", "Measurement::PoseList");
	bind_measurement_type<Ubitrack::Measurement::ErrorPoseList>(m, "ErrorPoseList", "Measurement::ErrorPoseList");
	bind_measurement_type<Ubitrack::Measurement::ErrorPositionList2>(m, "ErrorPositionList2", "Measurement::ErrorPositionList2");
	bind_measurement_type<Ubitrack::Measurement::ErrorPositionList>(m, "ErrorPositionList", "Measurement::ErrorPositionList");
	//RotationVelocity
	//CameraIntrinsics
}