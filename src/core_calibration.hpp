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

#include <complex>

#include "ubitrack_python/opaque_types.h"
#include "ubitrack_python/pyubitrack.h"

#include <boost/optional.hpp>
#include <boost/utility/typed_in_place_factory.hpp>

#include <istream>
#include <streambuf>
#include <iostream>

#include <utUtil/Exception.h>
#include <utMath/ErrorPose.h>
#include <utMath/ErrorVector.h>
#include <utMath/VectorFunctions.h>
#include <utMath/Stochastic/Average.h>
#include <utMath/Stochastic/Correlation.h>
#include <utMeasurement/Measurement.h>

#include <utAlgorithm/Projection.h>
#include <utAlgorithm/ToolTip/TipCalibration.h>
#include <utAlgorithm/PoseEstimation3D3D/AbsoluteOrientation.h>
// this include somehow causes problems on current OSX (maybe try on different platform/compiler)
// #include <utAlgorithm/PoseEstimation3D3D/CovarianceEstimation.h>
#include <utAlgorithm/PoseEstimation6D6D/TsaiLenz.h>

#include <queue> // std::queue
#include <map> // std::map
#include <tuple> // std::tuple
#include <vector>



Ubitrack::Math::Matrix< double, 4, 4 > projectionMatrix3x3ToOpenGL( double l, double r, double b, double t, double n, double f, Ubitrack::Math::Matrix< double, 3, 3 > m ) {
    return Ubitrack::Algorithm::projectionMatrixToOpenGL( l, r, b, t, n, f, m );
}

Ubitrack::Math::Matrix< double, 4, 4 > projectionMatrix3x4ToOpenGL( double l, double r, double b, double t, double n, double f, Ubitrack::Math::Matrix< double, 3, 4 > m ) {
    return Ubitrack::Algorithm::projectionMatrixToOpenGL( l, r, b, t, n, f, m );
}

Ubitrack::Math::Vector< double, 3 > tipCalibration( const std::vector< Ubitrack::Math::Pose > &poses) {
    Ubitrack::Math::Vector< double, 3 > pm;
    Ubitrack::Math::Vector< double, 3 > pw;
    Ubitrack::Algorithm::ToolTip::tipCalibration(poses, pm, pw);
    return pm;
}

Ubitrack::Math::Pose tipCalibrationPose( const std::vector< Ubitrack::Math::Pose > &poses) {
    Ubitrack::Math::Vector< double, 3 > pm;
    Ubitrack::Math::Vector< double, 3 > pw;
    Ubitrack::Algorithm::ToolTip::tipCalibration(poses, pm, pw);

    double length = norm_2(pm);
    Ubitrack::Math::Vector< double, 3 > refPos = -pm / length;
    Ubitrack::Math::Vector< double, 3 > errPos(0,0,1);

    // 1. Calc normalized correction axis
    Ubitrack::Math::Vector< double, 3 > axis = cross_product(errPos, refPos);
    // 2. Calc the correction angle
    double angle = acos (inner_prod(errPos, refPos) / ( norm_2(errPos) * norm_2(refPos)));
    // 3. Calc orientational correction. The axis will be normed by the constructor
    Ubitrack::Math::Quaternion corrRot( axis, angle);

    Ubitrack::Math::Pose p(corrRot, pm);
    return p;
}

Ubitrack::Math::Pose estimatePose6D_3D3D(const std::vector< Ubitrack::Math::Vector< double, 3> > &pointsA, const std::vector< Ubitrack::Math::Vector< double, 3> > &pointsB) {
    Ubitrack::Math::Pose resultPoseBA;

    if (pointsA.size() != pointsB.size()) {
        throw pybind11::value_error("Lists must contain the same number of elements");
    }

    bool success = Ubitrack::Algorithm::PoseEstimation3D3D::estimatePose6D_3D3D(pointsB, resultPoseBA, pointsA);
    if (!success) {
        throw std::runtime_error("Cannot estimate a result pose, some problem occured while performing a 3d-3d pose estimation (absolute orientation)");
    }
    return resultPoseBA;
}

/* compiler problem
Ubitrack::Math::ErrorPose estimatePose6D_3D3D_Error(const std::vector< Ubitrack::Math::Vector< double, 3> > &pointsA, const std::vector< Ubitrack::Math::Vector< double, 3> > &pointsB) {
    Ubitrack::Math::Pose resultPoseBA;

    if (pointsA.size() != pointsB.size()) {
        throw pybind11::value_error("Lists must contain the same number of elements");
    }

    bool success = Ubitrack::Algorithm::PoseEstimation3D3D::estimatePose6D_3D3D(pointsB, resultPoseBA, pointsA);
    if (!success) {
        throw std::runtime_error("Cannot estimate a result pose, some problem occured while performing a 3d-3d pose estimation (absolute orientation) ");
    }

    Ubitrack::Math::Matrix< double, 6, 6> cov = Ubitrack::Math::Matrix< double, 6, 6>::zeros();
    success = Ubitrack::Algorithm::PoseEstimation3D3D::estimatePose6DCovariance(pointsA.begin(), pointsA.end(), resultPoseBA, pointsB.begin(), pointsB.end(), cov);
    if (!success) {
        throw std::runtime_error("Cannot estimate a result, some problem occured while calculating the covariance");
    }

    Ubitrack::Math::ErrorPose ep(resultPoseBA, cov);
    return ep;
}
*/

Ubitrack::Math::Pose performHandEyeCalibration(const std::vector< Ubitrack::Math::Pose > &posesA, const std::vector< Ubitrack::Math::Pose > &posesB) {
    return Ubitrack::Algorithm::PoseEstimation6D6D::performHandEyeCalibration( posesA, posesB );

}

Ubitrack::Math::Vector<double, 3> tde_interpolate(const std::vector<Ubitrack::Measurement::Position>& data, Ubitrack::Measurement::Timestamp t){
    Ubitrack::Measurement::Position p1, p2;
    double factor = 1.0;
    for(int i=0; i<data.size();i++){
        if(data[i].time() > t){

            p1 = data[i-1];
            p2 = data[i];
            Ubitrack::Measurement::Timestamp eventDifference = p2.time() - p1.time();
            Ubitrack::Measurement::Timestamp timeDiff = t - p1.time();

            // check for timeout

            if ( eventDifference )
                factor = double( timeDiff ) / double( eventDifference );
            else
                factor = 1.0;

            break;

        }
    }
    return linearInterpolate(*p1, *p2, factor );
}

void tde_estimateTimeDelay(
        std::vector<double>::iterator data1First, std::vector<double>::iterator data1Last,
        std::vector<double>::iterator data2First, std::vector<double>::iterator data2Last,
        const long maxTimeOffsetInMs,
        std::map< int, std::vector< double > > &correlationData) {

    std::vector<double> data1(data1First, data1Last);

    for (int j = -maxTimeOffsetInMs; j < maxTimeOffsetInMs; j++) {
        std::vector<double> data2(data2First + j, data2Last + j);

        double corr = Ubitrack::Math::Stochastic::correlation(data1.begin(), data1.end(), data2.begin(), data2.end());
        correlationData[j].push_back(corr);
    }
}

/* This should really be a part of utCore as a function or class ... */
std::tuple<int, double> estimateTimeDelay(
        const std::vector< Ubitrack::Measurement::Position > &pointsA,
        const std::vector< Ubitrack::Measurement::Position > &pointsB,
        const long recordSizeInMs, const long maxTimeOffsetInMs, const long sliceSizeInMs, const float minSTDforMovement) {

    typedef std::vector<double>::iterator vec_iter;

    // check input data
    if(pointsA.size() > 0 && pointsB.size() > 0){
        bool avail1 = (pointsA.back().time() - pointsA.front().time()) >= recordSizeInMs*1000000LL;
        bool avail2 = (pointsB.back().time() - pointsB.front().time()) >= recordSizeInMs*1000000LL;
        if(!(avail1 && avail2)){
            throw pybind11::value_error("Data does not meet requirements.");
        }
    }

    // check if data is useable
    Ubitrack::Math::Stochastic::Average< Ubitrack::Math::ErrorVector<double, 3 > > averageData;
    std::vector< Ubitrack::Math::Vector3d > tmpData;
    for(int i=0;i < pointsA.size();i++){
        tmpData.push_back(*(pointsA.at(i)));
    }
    averageData = std::for_each( tmpData.begin(), tmpData.end(), averageData );
    Ubitrack::Math::ErrorVector<double, 3 > meanWithError = averageData.getAverage();

    if(meanWithError.getRMS() < minSTDforMovement){
        throw pybind11::value_error("Too little Movement.");
    }

    // get first common timestamp
    Ubitrack::Measurement::Timestamp startTime = std::max(pointsA.front().time(), pointsB.front().time());
    Ubitrack::Measurement::Timestamp endTime = std::min(pointsA.back().time(), pointsB.back().time());

    // interpolate data in ms steps
    std::vector<double>::size_type totalCount = (endTime - startTime) / 1000000L;
    std::vector<double> corr_data1;
    corr_data1.reserve(totalCount);
    std::vector<double> corr_data2;
    corr_data2.reserve(totalCount);

    for(std::vector<double>::size_type i=0;i<totalCount;i++){
        Ubitrack::Measurement::Timestamp t = startTime+i*1000000LL;
        Ubitrack::Math::Vector3d p1 = tde_interpolate(pointsA, t);
        Ubitrack::Math::Vector3d p2 = tde_interpolate(pointsB, t);

        // reduce to one dimension
        corr_data1.push_back( boost::numeric::ublas::norm_2(p1) );
        corr_data2.push_back( boost::numeric::ublas::norm_2(p2) );
    }


    // select slices and estimate time delay
    int countSlices = (corr_data1.size() - maxTimeOffsetInMs*2) / sliceSizeInMs;

    vec_iter data1First = corr_data1.begin();
    vec_iter data2First = corr_data2.begin();

    std::map< int, std::vector< double > > correlationData;

    for(int i=0; i<countSlices; i++){
        data1First = data1First + maxTimeOffsetInMs;
        data2First = data2First + maxTimeOffsetInMs;

        tde_estimateTimeDelay(data1First, data1First+sliceSizeInMs, data2First, data2First+sliceSizeInMs, maxTimeOffsetInMs, correlationData);
    }

    int offset=0;
    double correlation=0;

    for(std::map< int, std::vector< double > >::iterator it = correlationData.begin();it != correlationData.end();++it){


        Ubitrack::Math::Stochastic::Average<double> average;
        average = std::for_each( it->second.begin(), it->second.end(), average );

        double value = average.getAverage();
        if(correlation < value)
        {
            correlation = value;
            offset = it->first;
        }

    }
    return std::make_tuple(offset, correlation);
}


void bind_utCalibration(py::module& m)
{
    m.def("projectionMatrix3x3ToOpenGL", &projectionMatrix3x3ToOpenGL);
    m.def("projectionMatrix3x4ToOpenGL", &projectionMatrix3x4ToOpenGL);
    m.def("tipCalibration", &tipCalibration);
    m.def("tipCalibrationPose", &tipCalibrationPose);
    m.def("absoluteOrientation", &estimatePose6D_3D3D);
    // m.def("absoluteOrientationError", &estimatePose6D_3D3D_Error);
    m.def("performHandEyeCalibration", &performHandEyeCalibration);
    m.def("estimateTimeDelay", &estimateTimeDelay);
}