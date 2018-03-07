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

#include <utHaptics/Function/PhantomForwardKinematics.h>
#include <utHaptics/PhantomLMCalibration.h>
#include <utHaptics/PhantomLMGimbalCalibration.h>


Ubitrack::Math::Pose computePhantomForwardKinematicsPose(
        const Ubitrack::Math::Vector< double, 3 > &joint_angles,
        const Ubitrack::Math::Vector< double, 3 > &gimbal_angles,
        const Ubitrack::Math::Matrix< double, 3, 3 > &ja_correction,
        const Ubitrack::Math::Matrix< double, 3, 3 > &ga_correction,
        const Ubitrack::Math::Vector< double, 2 > &joint_lengths,
        const Ubitrack::Math::Vector< double, 3 > &origin_calib) {

    return Ubitrack::Haptics::Function::computePhantomForwardKinematicsPose(joint_angles, gimbal_angles, ja_correction, ga_correction, joint_lengths, origin_calib);
}

Ubitrack::Math::Matrix< double, 3, 3 > computePhantomLMCalibration(
        const std::vector< Ubitrack::Math::Vector< double, 3> > &joint_angles,
        const std::vector< Ubitrack::Math::Vector< double, 3> > &points,
        Ubitrack::Math::Vector< double, 2 > &joint_lengths,
        Ubitrack::Math::Vector< double, 3 > &origin_calib,
        const float optimizationStepSize,
        const float optimizationStepFactor ) {

    return Ubitrack::Haptics::computePhantomLMCalibration(joint_angles, points, joint_lengths( 0 ), joint_lengths( 1 ), origin_calib, (double)optimizationStepSize, (double)optimizationStepFactor);
}

Ubitrack::Math::Matrix< double, 3, 3 > computePhantomLMGimbalCalibration(
        const std::vector< Ubitrack::Math::Vector< double, 3> > &joint_angles,
        const std::vector< Ubitrack::Math::Vector< double, 3> > &gimbal_angles,
        const std::vector< Ubitrack::Math::Vector< double, 3> > &zref,
        const Ubitrack::Math::Matrix< double, 3, 3 > &ja_correction,
        Ubitrack::Math::Vector< double, 2 > &joint_lengths,
        Ubitrack::Math::Vector< double, 3 > &origin_calib,
        const float optimizationStepSize,
        const float optimizationStepFactor) {

    return Ubitrack::Haptics::computePhantomLMGimbalCalibration(joint_angles, gimbal_angles, zref, joint_lengths( 0 ), joint_lengths( 1 ), ja_correction, origin_calib, (double)optimizationStepSize, (double)optimizationStepFactor);
}

void bind_utHapticCalibrationMain(py::module& m)
{
    m.def("computePhantomLMCalibration", &computePhantomLMCalibration);
    m.def("computePhantomLMGimbalCalibration", &computePhantomLMGimbalCalibration);
    m.def("computePhantomForwardKinematicsPose", &computePhantomForwardKinematicsPose);

}