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
#include "ubitrack_python/pystreambuf.h"

#include <boost/optional.hpp>
#include <boost/utility/typed_in_place_factory.hpp>

#include <boost/filesystem.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp>

#include <istream>
#include <iostream>
#include <fstream>

#include <utMeasurement/Measurement.h>

#include <utUtil/Logging.h>
#include <utUtil/Exception.h>
#include <utUtil/CalibFile.h>

// read calib files
template< class T >
T readUtCalibFile(const std::string& sFile) {
	T value;
	Ubitrack::Util::readCalibFile< T >(sFile, value);
	return value;
}

template< class T >
typename T::value_type readUtCalibFileDropMeasurement(const std::string& sFile) {
	typename T::value_type value;
	Ubitrack::Util::readCalibFileDropMeasurement< typename T::value_type >(sFile, value);
	return value;
}


// read calib files
template< class T >
void writeUtCalibFile(const std::string& sFile, T& value) {
	Ubitrack::Util::writeCalibFile(sFile, value);
}

template< class EventType >
class EventStreamReader {
public:
	EventStreamReader( std::istream & s) {
		parse(s);
	}
	EventStreamReader( const std::string& filename) {
		// create ifstream
		std::ifstream stream( filename.c_str() );
		if ( !stream.good() )
			throw std::runtime_error( "Could not open file " + filename + " for reading" );
		parse(stream);
	}

	void parse(std::istream & s) {
		try {
			boost::archive::text_iarchive archive( s );

			// could be improved by only loading the requested event instead of all during init.
			// read contents until end-of-file exception
			try
			{
				while ( true )
				{
					EventType e( boost::shared_ptr< typename EventType::value_type >( new typename EventType::value_type() ) );
					std::string dummy; // for newline character in archive
					archive >> dummy >> e;
					data.push_back( e );
				}
			}
			catch (const boost::archive::archive_exception& e) {
				switch (e.code) {
					case boost::archive::archive_exception::unregistered_class:
						throw std::runtime_error("Boost::archive error: unregistered_class");
						break;
                    case boost::archive::archive_exception::invalid_signature:
                            throw std::runtime_error("Boost::archive error: invalid_signature");
                            break;
                    case boost::archive::archive_exception::unsupported_version:
                            throw std::runtime_error("Boost::archive error: unsupported_version");
                            break;
                    case boost::archive::archive_exception::pointer_conflict:
                            throw std::runtime_error("Boost::archive error: pointer_conflict");
                            break;
                    case boost::archive::archive_exception::incompatible_native_format:
                            throw std::runtime_error("Boost::archive error: incompatible_native_format");
                            break;
                    case boost::archive::archive_exception::array_size_too_short:
                            throw std::runtime_error("Boost::archive error: array_size_too_short");
                            break;
                    case boost::archive::archive_exception::input_stream_error:
                            // corresponds to end of stream, so this is ignored
                            //throw std::runtime_error("Boost::archive error: input_stream_error");
                            break;
                    case boost::archive::archive_exception::invalid_class_name:
                            throw std::runtime_error("Boost::archive error: invalid_class_name");
                            break;
                    case boost::archive::archive_exception::unsupported_class_version:
                            throw std::runtime_error("Boost::archive error: unsupported_class_version");
                            break;
                    case boost::archive::archive_exception::multiple_code_instantiation:
                            throw std::runtime_error("Boost::archive error: multiple_code_instantiation");
                            break;
                    case boost::archive::archive_exception::output_stream_error:
                            throw std::runtime_error("Boost::archive error: output_stream_error");
                            break;
					default:
                           throw std::runtime_error(e.what());
                           break;
				}

			}
			catch (const std::exception& ex) {
				// end of archive
				if (std::string(ex.what()).compare("input stream error") != 0 ) {
					throw std::runtime_error(ex.what());
				}
			} catch (const std::string& ex) {
				throw std::runtime_error(ex.c_str());
			} catch (...) {
				throw std::runtime_error("EventStreamReader unknown exception");
			}

		}
		catch (const std::exception& ex) {
			throw std::runtime_error(ex.what());
		} catch (const std::string& ex) {
			throw std::runtime_error(ex.c_str());
		} catch (...) {
			throw std::runtime_error("Init iarchive unknown exception");
		}


	}

	virtual ~EventStreamReader() {}

	std::vector< EventType > values() {
		return data;
	}

protected:
        std::vector< EventType > data;
};



void bind_utUtil(py::module& m)
{
	py::register_exception<Ubitrack::Util::Exception>(m, "UbitrackException");

	m.def("initLogging", &Ubitrack::Util::initLogging, py::arg("sConfigFile") = "log4cpp.conf");

    // EventStreamReaders
    py::class_< EventStreamReader< Ubitrack::Measurement::Pose > >(m, "PoseStreamReader")
		.def(py::init<const std::string & >())
		.def(py::init<std::istream & >())
		.def("values", &EventStreamReader<Ubitrack::Measurement::Pose >::values)
		;

    py::class_< EventStreamReader< Ubitrack::Measurement::ErrorPose > >(m, "ErrorPoseStreamReader")
		.def(py::init<const std::string & >())
		.def(py::init<std::istream & >())
		.def("values", &EventStreamReader<Ubitrack::Measurement::ErrorPose >::values)
		;

    py::class_< EventStreamReader< Ubitrack::Measurement::ErrorPosition > >(m, "ErrorPositionStreamReader")
		.def(py::init<const std::string & >())
		.def(py::init<std::istream & >())
		.def("values", &EventStreamReader<Ubitrack::Measurement::ErrorPosition >::values)
		;

    py::class_< EventStreamReader< Ubitrack::Measurement::Rotation > >(m, "RotationStreamReader")
		.def(py::init<const std::string & >())
		.def(py::init<std::istream & >())
		.def("values", &EventStreamReader<Ubitrack::Measurement::Rotation >::values)
		;

    py::class_< EventStreamReader< Ubitrack::Measurement::Position > >(m, "PositionStreamReader")
		.def(py::init<const std::string & >())
		.def(py::init<std::istream & >())
		.def("values", &EventStreamReader<Ubitrack::Measurement::Position >::values)
		;

    py::class_< EventStreamReader< Ubitrack::Measurement::Position2D > >(m, "Position2DStreamReader")
		.def(py::init<const std::string & >())
		.def(py::init<std::istream & >())
		.def("values", &EventStreamReader<Ubitrack::Measurement::Position2D >::values)
		;

    py::class_< EventStreamReader< Ubitrack::Measurement::Matrix3x3 > >(m, "Matrix3x3StreamReader")
		.def(py::init<const std::string & >())
		.def(py::init<std::istream & >())
		.def("values", &EventStreamReader<Ubitrack::Measurement::Matrix3x3 >::values)
		;

    py::class_< EventStreamReader< Ubitrack::Measurement::Matrix3x4 > >(m, "Matrix3x4StreamReader")
		.def(py::init<const std::string & >())
		.def(py::init<std::istream & >())
		.def("values", &EventStreamReader<Ubitrack::Measurement::Matrix3x4 >::values)
		;

    py::class_< EventStreamReader< Ubitrack::Measurement::Matrix4x4 > >(m, "Matrix4x4StreamReader")
		.def(py::init<const std::string & >())
		.def(py::init<std::istream & >())
		.def("values", &EventStreamReader<Ubitrack::Measurement::Matrix4x4 >::values)
		;

    py::class_< EventStreamReader< Ubitrack::Measurement::PositionList > >(m, "PositionListStreamReader")
		.def(py::init<const std::string & >())
		.def(py::init<std::istream & >())
		.def("values", &EventStreamReader<Ubitrack::Measurement::PositionList >::values)
		;

    py::class_< EventStreamReader< Ubitrack::Measurement::PositionList2 > >(m, "PositionList2StreamReader")
		.def(py::init<const std::string & >())
		.def(py::init<std::istream & >())
		.def("values", &EventStreamReader<Ubitrack::Measurement::PositionList2 >::values)
		;

    py::class_< EventStreamReader< Ubitrack::Measurement::PoseList > >(m, "PoseListStreamReader")
		.def(py::init<const std::string & >())
		.def(py::init<std::istream & >())
		.def("values", &EventStreamReader<Ubitrack::Measurement::PoseList >::values)
		;

//    py::class_< EventStreamReader< Ubitrack::Measurement::RotationVelocity > >("RotationVelocityStreamReader")
//		.def(py::init<const std::string & >())
//		.def(py::init<std::istream & >())
//		.def("values", &EventStreamReader<Ubitrack::Measurement::RotationVelocity >::values)
//		;
//
//    py::class_< EventStreamReader< Ubitrack::Measurement::Image > >("ImageStreamReader")
//		.def(py::init<const std::string & >())
//		.def(py::init<std::istream & >())
//		.def("values", &EventStreamReader<Ubitrack::Measurement::Image >::values)
//		;


    m.def("readCalibMeasurementDistance", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::Distance > );
    m.def("readCalibMeasurementPose", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::Pose > );
    m.def("readCalibMeasurementErrorPose", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::ErrorPose > );
    m.def("readCalibMeasurementErrorPosition", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::ErrorPosition > );
    m.def("readCalibMeasurementPosition", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::Position > );
    m.def("readCalibMeasurementRotation", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::Rotation > );
    m.def("readCalibMeasurementMatrix3x3", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::Matrix3x3 > );
    m.def("readCalibMeasurementMatrix3x4", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::Matrix3x4 > );
    m.def("readCalibMeasurementMatrix4x4", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::Matrix4x4 > );
    m.def("readCalibMeasurementVector4D", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::Vector4D > );
    m.def("readCalibMeasurementVector8D", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::Vector8D > );
    m.def("readCalibMeasurementPositionList", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::PositionList > );
    m.def("readCalibMeasurementCameraIntrinsics", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::CameraIntrinsics > );
    m.def("readCalibMeasurementPositionList2", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::PositionList2 > );
    m.def("readCalibMeasurementPoseList", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::PoseList > );
    m.def("readCalibMeasurementDistanceList", &readUtCalibFileDropMeasurement< Ubitrack::Measurement::DistanceList > );

    m.def("readCalibDistance", &readUtCalibFile< Ubitrack::Math::Scalar< double > >);
    m.def("readCalibPose", &readUtCalibFile< Ubitrack::Math::Pose >);
    m.def("readCalibErrorPose", &readUtCalibFile< Ubitrack::Math::ErrorPose >);
    m.def("readCalibErrorPosition", &readUtCalibFile< Ubitrack::Math::ErrorVector< double, 3 > >);
    m.def("readCalibPosition", &readUtCalibFile< Ubitrack::Math::Vector3d >);
    m.def("readCalibRotation", &readUtCalibFile< Ubitrack::Math::Quaternion >);
    m.def("readCalibMatrix3x3", &readUtCalibFile< Ubitrack::Math::Matrix3x3d >);
    m.def("readCalibMatrix3x4", &readUtCalibFile< Ubitrack::Math::Matrix3x4d >);
    m.def("readCalibMatrix4x4", &readUtCalibFile< Ubitrack::Math::Matrix4x4d >);
    m.def("readCalibVector4D", &readUtCalibFile< Ubitrack::Math::Vector4d >);
    m.def("readCalibVector8D", &readUtCalibFile< Ubitrack::Math::Vector< double, 8 > >);
    m.def("readCalibCameraIntrinsics", &readUtCalibFile< Ubitrack::Math::CameraIntrinsics<double> >);
    m.def("readCalibPositionList", &readUtCalibFile< std::vector< Ubitrack::Math::Vector3d > >);
    m.def("readCalibPositionList2", &readUtCalibFile< std::vector< Ubitrack::Math::Vector2d > >);
    m.def("readCalibPoseList", &readUtCalibFile< std::vector< Ubitrack::Math::Pose > >);
    m.def("readCalibDistanceList", &readUtCalibFile< std::vector< Ubitrack::Math::Scalar< double > > >);



    m.def("writeCalibMeasurementDistance", &writeUtCalibFile< Ubitrack::Measurement::Distance >);
    m.def("writeCalibMeasurementPose", &writeUtCalibFile< Ubitrack::Measurement::Pose >);
    m.def("writeCalibMeasurementErrorPose", &writeUtCalibFile< Ubitrack::Measurement::ErrorPose >);
    m.def("writeCalibMeasurementErrorPosition", &writeUtCalibFile< Ubitrack::Measurement::ErrorPosition >);
    m.def("writeCalibMeasurementPosition", &writeUtCalibFile< Ubitrack::Measurement::Position >);
    m.def("writeCalibMeasurementRotation", &writeUtCalibFile< Ubitrack::Measurement::Rotation >);
    m.def("writeCalibMeasurementMatrix3x3", &writeUtCalibFile< Ubitrack::Measurement::Matrix3x3 >);
    m.def("writeCalibMeasurementMatrix3x4", &writeUtCalibFile< Ubitrack::Measurement::Matrix3x4 >);
    m.def("writeCalibMeasurementMatrix4x4", &writeUtCalibFile< Ubitrack::Measurement::Matrix4x4 >);
    m.def("writeCalibMeasurementVector4D", &writeUtCalibFile< Ubitrack::Measurement::Vector4D >);
    m.def("writeCalibMeasurementVector8D", &writeUtCalibFile< Ubitrack::Measurement::Vector8D >);
    m.def("writeCalibMeasurementCameraIntrinsics", &writeUtCalibFile< Ubitrack::Measurement::CameraIntrinsics >);
    m.def("writeCalibMeasurementPositionList", &writeUtCalibFile< Ubitrack::Measurement::PositionList >);
    m.def("writeCalibMeasurementPositionList2", &writeUtCalibFile< Ubitrack::Measurement::PositionList2 >);
    m.def("writeCalibMeasurementPoseList", &writeUtCalibFile< Ubitrack::Measurement::PoseList >);
    m.def("writeCalibMeasurementDistanceList", &writeUtCalibFile< Ubitrack::Measurement::DistanceList >);

    m.def("writeCalibDistance", &writeUtCalibFile< Ubitrack::Math::Scalar< double > >);
    m.def("writeCalibPose", &writeUtCalibFile< Ubitrack::Math::Pose >);
    m.def("writeCalibErrorPose", &writeUtCalibFile< Ubitrack::Math::ErrorPose >);
    m.def("writeCalibErrorPosition", &writeUtCalibFile< Ubitrack::Math::ErrorVector< double, 3 > >);
    m.def("writeCalibPosition", &writeUtCalibFile< Ubitrack::Math::Vector3d >);
    m.def("writeCalibRotation", &writeUtCalibFile< Ubitrack::Math::Quaternion >);
    m.def("writeCalibMatrix3x3", &writeUtCalibFile< Ubitrack::Math::Matrix3x3d >);
    m.def("writeCalibMatrix3x4", &writeUtCalibFile< Ubitrack::Math::Matrix3x4d >);
    m.def("writeCalibMatrix4x4", &writeUtCalibFile< Ubitrack::Math::Matrix4x4d >);
    m.def("writeCalibVector4D", &writeUtCalibFile< Ubitrack::Math::Vector4d >);
    m.def("writeCalibVector8D", &writeUtCalibFile< Ubitrack::Math::Vector< double, 8 > >);
    m.def("writeCalibCameraIntrinsics", &writeUtCalibFile< Ubitrack::Math::CameraIntrinsics<double> >);
    m.def("writeCalibPositionList", &writeUtCalibFile< std::vector< Ubitrack::Math::Vector3d > >);
    m.def("writeCalibPositionList2", &writeUtCalibFile< std::vector< Ubitrack::Math::Vector2d > >);
    m.def("writeCalibPoseList", &writeUtCalibFile< std::vector< Ubitrack::Math::Pose > >);
    m.def("writeCalibDistanceList", &writeUtCalibFile< std::vector< Ubitrack::Math::Scalar< double > > >);

}