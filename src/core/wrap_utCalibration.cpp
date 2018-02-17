#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#ifndef HAVE_BOOST_NUMPY
#include <boost/python/numpy.hpp>
#else
#include <boost/numpy.hpp>
#endif

#include <numpy/arrayobject.h>
#include <complex>


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
#include <utAlgorithm/PoseEstimation3D3D/CovarianceEstimation.h>
#include <utAlgorithm/PoseEstimation6D6D/TsaiLenz.h>


#include <queue> // std::queue
#include <map> // std::map


using namespace Ubitrack;

namespace bp = boost::python;
#ifndef HAVE_BOOST_NUMPY
namespace bn = boost::python::numpy;
#else
namespace bn = boost::numpy;
#endif
namespace ublas = boost::numeric::ublas;

namespace {


	Math::Matrix< double, 4, 4 > projectionMatrix3x3ToOpenGL( double l, double r, double b, double t, double n, double f, Math::Matrix< double, 3, 3 > m ) {
		return Algorithm::projectionMatrixToOpenGL( l, r, b, t, n, f, m );
	}

	Math::Matrix< double, 4, 4 > projectionMatrix3x4ToOpenGL( double l, double r, double b, double t, double n, double f, Math::Matrix< double, 3, 4 > m ) {
		return Algorithm::projectionMatrixToOpenGL( l, r, b, t, n, f, m );
	}

	Math::Vector< double, 3 > tipCalibration( const std::vector< Ubitrack::Math::Pose > &poses) {
		Ubitrack::Math::Vector< double, 3 > pm;
		Ubitrack::Math::Vector< double, 3 > pw;
		Algorithm::ToolTip::tipCalibration(poses, pm, pw);
		return pm;
	}

	Math::Pose tipCalibrationPose( const std::vector< Ubitrack::Math::Pose > &poses) {
		Ubitrack::Math::Vector< double, 3 > pm;
		Ubitrack::Math::Vector< double, 3 > pw;
		Algorithm::ToolTip::tipCalibration(poses, pm, pw);

		double length = norm_2(pm);
		Math::Vector< double, 3 > refPos = -pm / length;
		Math::Vector< double, 3 > errPos(0,0,1);

		// 1. Calc normalized correction axis
		Math::Vector< double, 3 > axis = cross_product(errPos, refPos);
		// 2. Calc the correction angle
		double angle = acos (inner_prod(errPos, refPos) / ( norm_2(errPos) * norm_2(refPos)));
		// 3. Calc orientational correction. The axis will be normed by the constructor
		Math::Quaternion corrRot( axis, angle);

		Math::Pose p(corrRot, pm);
		return p;
	}

	Math::Pose estimatePose6D_3D3D(const std::vector< Ubitrack::Math::Vector< double, 3> > &pointsA, const std::vector< Ubitrack::Math::Vector< double, 3> > &pointsB) {
		Math::Pose resultPoseBA;

		if (pointsA.size() != pointsB.size()) {
			PyErr_SetString(PyExc_ValueError, "Lists must contain the same number of elements");
			bp::throw_error_already_set();
		}

		bool success = Algorithm::PoseEstimation3D3D::estimatePose6D_3D3D(pointsB, resultPoseBA, pointsA);
		if (!success) {
			UBITRACK_THROW( "Cannot estimate a result pose, some problem occured while performing a 3d-3d pose estimation (absolute orientation)");
		}
		return resultPoseBA;
	}

	Math::ErrorPose estimatePose6D_3D3D_Error(const std::vector< Ubitrack::Math::Vector< double, 3> > &pointsA, const std::vector< Ubitrack::Math::Vector< double, 3> > &pointsB) {
		Math::Pose resultPoseBA;

		if (pointsA.size() != pointsB.size()) {
			PyErr_SetString(PyExc_ValueError, "Lists must contain the same number of elements");
			bp::throw_error_already_set();
		}

		bool success = Algorithm::PoseEstimation3D3D::estimatePose6D_3D3D(pointsB, resultPoseBA, pointsA);
		if (!success) {
			UBITRACK_THROW( "Cannot estimate a result pose, some problem occured while performing a 3d-3d pose estimation (absolute orientation) ");
		}

		Ubitrack::Math::Matrix< double, 6, 6> cov = Ubitrack::Math::Matrix< double, 6, 6>::zeros();
		success = Algorithm::PoseEstimation3D3D::estimatePose6DCovariance(pointsA.begin(), pointsA.end(), resultPoseBA, pointsB.begin(), pointsB.end(), cov);
		if (!success) {
			UBITRACK_THROW( "Cannot estimate a result, some problem occured while calculating the covariance");
		}

		Ubitrack::Math::ErrorPose ep(resultPoseBA, cov);
		return ep;
	}


	Math::Pose performHandEyeCalibration(const std::vector< Ubitrack::Math::Pose > &posesA, const std::vector< Ubitrack::Math::Pose > &posesB) {
		return Algorithm::PoseEstimation6D6D::performHandEyeCalibration( posesA, posesB );

	}



	Math::Vector<double, 3> tde_interpolate(const std::vector<Measurement::Position>& data, Measurement::Timestamp t){
		Measurement::Position p1, p2;
		double factor = 1.0;
		for(int i=0; i<data.size();i++){
			if(data[i].time() > t){

				p1 = data[i-1];
				p2 = data[i];
				Measurement::Timestamp eventDifference = p2.time() - p1.time();
				Measurement::Timestamp timeDiff = t - p1.time();

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

			double corr = Math::Stochastic::correlation(data1.begin(), data1.end(), data2.begin(), data2.end());
			correlationData[j].push_back(corr);
		}
	}

	/* This should really be a part of utCore as a function or class ... */
	bp::tuple estimateTimeDelay(
			const std::vector< Ubitrack::Measurement::Position > &pointsA,
			const std::vector< Ubitrack::Measurement::Position > &pointsB,
			const long recordSizeInMs, const long maxTimeOffsetInMs, const long sliceSizeInMs, const float minSTDforMovement) {

		typedef std::vector<double>::iterator vec_iter;

		// check input data
		if(pointsA.size() > 0 && pointsB.size() > 0){
			bool avail1 = (pointsA.back().time() - pointsA.front().time()) >= recordSizeInMs*1000000LL;
			bool avail2 = (pointsB.back().time() - pointsB.front().time()) >= recordSizeInMs*1000000LL;
			if(!(avail1 && avail2)){
				PyErr_SetString(PyExc_ValueError, "Data does not meet requirements.");
				bp::throw_error_already_set();
			}
		}

		// check if data is useable
		Math::Stochastic::Average<Math::ErrorVector<double, 3 > > averageData;
		std::vector< Math::Vector3d > tmpData;
		for(int i=0;i < pointsA.size();i++){
			tmpData.push_back(*(pointsA.at(i)));
		}
		averageData = std::for_each( tmpData.begin(), tmpData.end(), averageData );
		Math::ErrorVector<double, 3 > meanWithError = averageData.getAverage();

		if(meanWithError.getRMS() < minSTDforMovement){
			PyErr_SetString(PyExc_ValueError, "Too little Movement.");
			bp::throw_error_already_set();
		}

		// get first common timestamp
		Measurement::Timestamp startTime = std::max(pointsA.front().time(), pointsB.front().time());
		Measurement::Timestamp endTime = std::min(pointsA.back().time(), pointsB.back().time());

		// interpolate data in ms steps
		std::vector<double>::size_type totalCount = (endTime - startTime) / 1000000L;
		std::vector<double> corr_data1;
		corr_data1.reserve(totalCount);
		std::vector<double> corr_data2;
		corr_data2.reserve(totalCount);

		for(std::vector<double>::size_type i=0;i<totalCount;i++){
			Measurement::Timestamp t = startTime+i*1000000LL;
			Math::Vector3d p1 = tde_interpolate(pointsA, t);
			Math::Vector3d p2 = tde_interpolate(pointsB, t);

			// reduce to one dimension
			corr_data1.push_back( ublas::norm_2(p1) );
			corr_data2.push_back( ublas::norm_2(p2) );
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


			Math::Stochastic::Average<double> average;
			average = std::for_each( it->second.begin(), it->second.end(), average );

			double value = average.getAverage();
			if(correlation < value)
			{
				correlation = value;
				offset = it->first;
			}

		}
		return bp::make_tuple(offset, correlation);
	}


} // end anon ns


BOOST_PYTHON_MODULE(_utcalibration)
{
	// initialize boost.numpy
	bn::initialize();

	bp::def("projectionMatrix3x3ToOpenGL", &projectionMatrix3x3ToOpenGL);
	bp::def("projectionMatrix3x4ToOpenGL", &projectionMatrix3x4ToOpenGL);
	bp::def("tipCalibration", &tipCalibration);
	bp::def("tipCalibrationPose", &tipCalibrationPose);
	bp::def("absoluteOrientation", &estimatePose6D_3D3D);
	bp::def("absoluteOrientationError", &estimatePose6D_3D3D_Error);
	bp::def("performHandEyeCalibration", &performHandEyeCalibration);
	bp::def("estimateTimeDelay", &estimateTimeDelay);
}
