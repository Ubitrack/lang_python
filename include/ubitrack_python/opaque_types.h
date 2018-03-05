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
 * Ubitrack Python Bindings - Opaque Types.
 * @author Ulrich Eck <ulrich.eck@tum.de>
 */

#ifndef UBITRACK_PYTHON_OPAQUE_TYPES_H_
#define UBITRACK_PYTHON_OPAQUE_TYPES_H_

#include <ubitrack_python/pyubitrack.h>
#include <pybind11/stl_bind.h>
#include <vector>
#include <map>

#include <utMath/Scalar.h>
#include <utMath/Vector.h>
#include <utMath/ErrorVector.h>
#include <utMath/Matrix.h>
#include <utMath/Quaternion.h>
#include <utMath/Pose.h>
#include <utMath/ErrorPose.h>
#include <utMath/RotationVelocity.h>
#include <utMath/CameraIntrinsics.h>

// All uses of PYBIND11_MAKE_OPAQUE need to be in this common header to avoid ODR
// violations.
namespace UbitrackMathTypes {
typedef Ubitrack::Math::Vector<double, 2> Vector2d;
typedef Ubitrack::Math::Vector<double, 3> Vector3d;
typedef Ubitrack::Math::Vector<double, 4> Vector4d;
typedef Ubitrack::Math::Vector<double, 5> Vector5d;
typedef Ubitrack::Math::Vector<double, 6> Vector6d;
typedef Ubitrack::Math::Vector<double, 7> Vector7d;
typedef Ubitrack::Math::Vector<double, 8> Vector8d;

typedef Ubitrack::Math::Vector<float, 2> Vector2f;
typedef Ubitrack::Math::Vector<float, 3> Vector3f;
typedef Ubitrack::Math::Vector<float, 4> Vector4f;
typedef Ubitrack::Math::Vector<float, 5> Vector5f;
typedef Ubitrack::Math::Vector<float, 6> Vector6f;
typedef Ubitrack::Math::Vector<float, 7> Vector7f;
typedef Ubitrack::Math::Vector<float, 8> Vector8f;

typedef Ubitrack::Math::ErrorVector<double, 2> ErrorVector2d;
typedef Ubitrack::Math::ErrorVector<double, 3> ErrorVector3d;
typedef Ubitrack::Math::ErrorVector<double, 7> ErrorVector7d;

typedef Ubitrack::Math::Matrix<double, 2, 2> Matrix22d;
typedef Ubitrack::Math::Matrix<double, 3, 3> Matrix33d;
typedef Ubitrack::Math::Matrix<double, 3, 4> Matrix34d;
typedef Ubitrack::Math::Matrix<double, 4, 4> Matrix44d;
typedef Ubitrack::Math::Matrix<double, 6, 6> Matrix66d;

typedef Ubitrack::Math::Matrix<float, 2, 2> Matrix22f;
typedef Ubitrack::Math::Matrix<float, 3, 3> Matrix33f;
typedef Ubitrack::Math::Matrix<float, 3, 4> Matrix34f;
typedef Ubitrack::Math::Matrix<float, 4, 4> Matrix44f;
typedef Ubitrack::Math::Matrix<float, 6, 6> Matrix66f;
}

PYBIND11_MAKE_OPAQUE(std::vector<Ubitrack::Math::Scalar<unsigned long>>);
PYBIND11_MAKE_OPAQUE(std::vector<Ubitrack::Math::Scalar<int>>);
PYBIND11_MAKE_OPAQUE(std::vector<Ubitrack::Math::Scalar<double>>);

PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Vector2d>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Vector3d>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Vector4d>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Vector5d>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Vector6d>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Vector7d>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Vector8d>);

PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Vector2f>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Vector3f>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Vector4f>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Vector5f>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Vector6f>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Vector7f>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Vector8f>);

PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::ErrorVector2d>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::ErrorVector3d>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::ErrorVector7d>);

PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Matrix22d>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Matrix33d>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Matrix34d>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Matrix44d>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Matrix66d>);

PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Matrix22f>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Matrix33f>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Matrix34f>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Matrix44f>);
PYBIND11_MAKE_OPAQUE(std::vector<UbitrackMathTypes::Matrix66f>);

PYBIND11_MAKE_OPAQUE(std::vector<Ubitrack::Math::Pose>);
PYBIND11_MAKE_OPAQUE(std::vector<Ubitrack::Math::ErrorPose>);

PYBIND11_MAKE_OPAQUE(std::vector<Ubitrack::Math::RotationVelocity>);

PYBIND11_MAKE_OPAQUE(std::vector<Ubitrack::Math::CameraIntrinsics<double>>);

#endif // UBITRACK_PYTHON_OPAQUE_TYPES_H_
