#include <pybind11/pybind11.h>

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

// #include <ubitrack_python/wrap_streambuf.h>

using namespace Ubitrack;
using namespace Ubitrack::Python;

namespace py = pybind11;

namespace {

void translateException( const Ubitrack::Util::Exception& e) {
    PyErr_SetString(PyExc_RuntimeError, e.what());
};



// read calib files
template< class T >
T readUtCalibFile(const std::string& sFile) {
	T value;
	Util::readCalibFile< T >(sFile, value);
	return value;
}

template< class T >
typename T::value_type readUtCalibFileDropMeasurement(const std::string& sFile) {
	typename T::value_type value;
	Util::readCalibFileDropMeasurement< typename T::value_type >(sFile, value);
	return value;
}


// read calib files
template< class T >
void writeUtCalibFile(const std::string& sFile, T& value) {
	Util::writeCalibFile(sFile, value);
}


template<class T>
struct register_measurement_vector {

	register_measurement_vector(char const* name) {
        bp::type_info info = bp::type_id<T>();
        const bp::converter::registration* reg = bp::converter::registry::query(info);
        if (reg == NULL)  {
            bp::class_< T >(name)
                .def(bp::vector_indexing_suite< T >() );
        } else if ((*reg).m_to_python == NULL) {
            bp::class_< T >(name)
                .def(bp::vector_indexing_suite< T >() );
        }
	}
};

} // end anon ns


template< class EventType >
class EventStreamReader {
public:
	EventStreamReader( streambuf& sb) {
		streambuf::istream file(sb);

		try {
			boost::archive::text_iarchive archive( file );

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
						PyErr_SetString(PyExc_RuntimeError, "Boost::archive error: unregistered_class");
						break;
                                        case boost::archive::archive_exception::invalid_signature:
                                                PyErr_SetString(PyExc_RuntimeError, "Boost::archive error: invalid_signature");
                                                break;
                                        case boost::archive::archive_exception::unsupported_version:
                                                PyErr_SetString(PyExc_RuntimeError, "Boost::archive error: unsupported_version");
                                                break;
                                        case boost::archive::archive_exception::pointer_conflict:
                                                PyErr_SetString(PyExc_RuntimeError, "Boost::archive error: pointer_conflict");
                                                break;
                                        case boost::archive::archive_exception::incompatible_native_format:
                                                PyErr_SetString(PyExc_RuntimeError, "Boost::archive error: incompatible_native_format");
                                                break;
                                        case boost::archive::archive_exception::array_size_too_short:
                                                PyErr_SetString(PyExc_RuntimeError, "Boost::archive error: array_size_too_short");
                                                break;
                                        case boost::archive::archive_exception::input_stream_error:
                                                // corresponds to end of stream, so this is ignored
                                                //PyErr_SetString(PyExc_RuntimeError, "Boost::archive error: input_stream_error");
                                                break;
                                        case boost::archive::archive_exception::invalid_class_name:
                                                PyErr_SetString(PyExc_RuntimeError, "Boost::archive error: invalid_class_name");
                                                break;
                                        case boost::archive::archive_exception::unsupported_class_version:
                                                PyErr_SetString(PyExc_RuntimeError, "Boost::archive error: unsupported_class_version");
                                                break;
                                        case boost::archive::archive_exception::multiple_code_instantiation:
                                                PyErr_SetString(PyExc_RuntimeError, "Boost::archive error: multiple_code_instantiation");
                                                break;
                                        case boost::archive::archive_exception::output_stream_error:
                                                PyErr_SetString(PyExc_RuntimeError, "Boost::archive error: output_stream_error");
                                                break;
					default:
                                               PyErr_SetString(PyExc_RuntimeError, e.what());
                                               break;
				}

			}
			catch (const std::exception& ex) {
				// end of archive
				if (std::string(ex.what()).compare("input stream error") != 0 ) {
					PyErr_SetString(PyExc_RuntimeError, ex.what());
				}
			} catch (const std::string& ex) {
				PyErr_SetString(PyExc_RuntimeError, ex.c_str());
			} catch (...) {
				PyErr_SetString(PyExc_RuntimeError, "EventStreamReader unknown exception");
			}

		}
		catch (const std::exception& ex) {
			PyErr_SetString(PyExc_RuntimeError, ex.what());
		} catch (const std::string& ex) {
			PyErr_SetString(PyExc_RuntimeError, ex.c_str());
		} catch (...) {
			PyErr_SetString(PyExc_RuntimeError, "Init iarchive unknown exception");
		}

	}
	virtual ~EventStreamReader() {}

	/*
	bp::list values() {
                bp::list result;
                for (typename std::vector< EventType >::iterator it = data.begin(); it != data.end(); ++it) {
                    result.append(*it);
                }
		return result;
	}
	*/

	std::vector< EventType > values() {
		return data;
	}

protected:
        std::vector< EventType > data;
};

BOOST_PYTHON_FUNCTION_OVERLOADS(logging_overloads, Ubitrack::Util::initLogging, 0, 1)

BOOST_PYTHON_MODULE(_ututil)
{
	bp::register_exception_translator<Ubitrack::Util::Exception>(translateException);

	bp::def("initLogging", &Ubitrack::Util::initLogging, logging_overloads());


	// provide streambuf support
	bp::class_<streambuf, boost::shared_ptr<streambuf>, boost::noncopyable> sb("streambuf", bp::no_init);
    sb.def(bp::init<bp::object&, std::size_t>((bp::arg("file"), bp::arg("buffer_size") = 0)));
    sb.def_readwrite("default_buffer_size", streambuf::default_buffer_size, "The default size of the buffer sitting "
                     "between a Python file object and a C++ stream.");

    bp::class_<std::ostream, boost::shared_ptr<std::ostream>, boost::noncopyable>("std_ostream", bp::no_init);
    bp::class_<ostream, boost::noncopyable, bp::bases<std::ostream> > os("ostream", bp::no_init);
    os.def(bp::init<bp::object&, std::size_t>((bp::arg("python_file_obj"), bp::arg("buffer_size") = 0)));
	// XXX fails on windows VS2010
    //os.def_readwrite("file",&ostream::get_original_file);

    bp::class_<std::istream, boost::shared_ptr<std::istream>, boost::noncopyable>("std_istream", bp::no_init);
    bp::class_<istream, boost::noncopyable, bp::bases<std::istream> > is("istream", bp::no_init);
    is.def(bp::init<bp::object&, std::size_t>((bp::arg("python_file_obj"), bp::arg("buffer_size") = 0)));
	// XXX fails on windows VS2010
    //is.def_readwrite("file",&ostream::get_original_file);

	// EventStreamIterators
    register_measurement_vector< std::vector< Measurement::Pose > >("PoseMeasurementIterator");
    register_measurement_vector< std::vector< Measurement::ErrorPose > >("ErrorPoseMeasurementIterator");
    register_measurement_vector< std::vector< Measurement::ErrorPosition > >("ErrorPositionMeasurementIterator");
    register_measurement_vector< std::vector< Measurement::Rotation > >("RotationMeasurementIterator");
    register_measurement_vector< std::vector< Measurement::Position > >("PositionMeasurementIterator");
    register_measurement_vector< std::vector< Measurement::Position2D > >("Position2DMeasurementIterator");
    register_measurement_vector< std::vector< Measurement::Matrix3x3 > >("Matrix3x3MeasurementIterator");
    register_measurement_vector< std::vector< Measurement::Matrix3x4 > >("Matrix3x4MeasurementIterator");
    register_measurement_vector< std::vector< Measurement::Matrix4x4 > >("Matrix4x4MeasurementIterator");
    register_measurement_vector< std::vector< Measurement::PositionList > >("PositionListMeasurementIterator");
    register_measurement_vector< std::vector< Measurement::PositionList2 > >("PositionList2MeasurementIterator");
    register_measurement_vector< std::vector< Measurement::PoseList > >("PoseListMeasurementIterator");


    // EventStreamReaders
    bp::class_< EventStreamReader< Measurement::Pose > >("PoseStreamReader", bp::init< streambuf& >())
		.def("values", &EventStreamReader<Measurement::Pose >::values)
		;

    bp::class_< EventStreamReader< Measurement::ErrorPose > >("ErrorPoseStreamReader", bp::init< streambuf& >())
		.def("values", &EventStreamReader<Measurement::ErrorPose >::values)
		;

    bp::class_< EventStreamReader< Measurement::ErrorPosition > >("ErrorPositionStreamReader", bp::init< streambuf& >())
		.def("values", &EventStreamReader<Measurement::ErrorPosition >::values)
		;

    bp::class_< EventStreamReader< Measurement::Rotation > >("RotationStreamReader", bp::init< streambuf& >())
		.def("values", &EventStreamReader<Measurement::Rotation >::values)
		;

    bp::class_< EventStreamReader< Measurement::Position > >("PositionStreamReader", bp::init< streambuf& >())
		.def("values", &EventStreamReader<Measurement::Position >::values)
		;

    bp::class_< EventStreamReader< Measurement::Position2D > >("Position2DStreamReader", bp::init< streambuf& >())
		.def("values", &EventStreamReader<Measurement::Position2D >::values)
		;

    bp::class_< EventStreamReader< Measurement::Matrix3x3 > >("Matrix3x3StreamReader", bp::init< streambuf& >())
		.def("values", &EventStreamReader<Measurement::Matrix3x3 >::values)
		;

    bp::class_< EventStreamReader< Measurement::Matrix3x4 > >("Matrix3x4StreamReader", bp::init< streambuf& >())
		.def("values", &EventStreamReader<Measurement::Matrix3x4 >::values)
		;

    bp::class_< EventStreamReader< Measurement::Matrix4x4 > >("Matrix4x4StreamReader", bp::init< streambuf& >())
		.def("values", &EventStreamReader<Measurement::Matrix4x4 >::values)
		;

    bp::class_< EventStreamReader< Measurement::PositionList > >("PositionListStreamReader", bp::init< streambuf& >())
		.def("values", &EventStreamReader<Measurement::PositionList >::values)
		;

    bp::class_< EventStreamReader< Measurement::PositionList2 > >("PositionList2StreamReader", bp::init< streambuf& >())
		.def("values", &EventStreamReader<Measurement::PositionList2 >::values)
		;

    bp::class_< EventStreamReader< Measurement::PoseList > >("PoseListStreamReader", bp::init< streambuf& >())
		.def("values", &EventStreamReader<Measurement::PoseList >::values)
		;

//    bp::class_< EventStreamReader< Measurement::RotationVelocity > >("RotationVelocityStreamReader", bp::init< streambuf& >())
//		.def("values", &EventStreamReader<Measurement::RotationVelocity >::values)
//		;
//
//    bp::class_< EventStreamReader< Measurement::Image > >("ImageStreamReader", bp::init< streambuf& >())
//		.def("values", &EventStreamReader<Measurement::Image >::values)
//		;



    // currently this returns the value only (dropping the measurement, because of technical problems with returning the "custom" shared pointers

    // segfaults ..
    //bp::def("readCalibMeasurementDistance", &readUtCalibFileDropMeasurement< Measurement::Distance > );

    bp::def("readCalibMeasurementPose", &readUtCalibFileDropMeasurement< Measurement::Pose > );
    bp::def("readCalibMeasurementErrorPose", &readUtCalibFileDropMeasurement< Measurement::ErrorPose > );
    bp::def("readCalibMeasurementErrorPosition", &readUtCalibFileDropMeasurement< Measurement::ErrorPosition > );
    bp::def("readCalibMeasurementPosition", &readUtCalibFileDropMeasurement< Measurement::Position > );
    bp::def("readCalibMeasurementRotation", &readUtCalibFileDropMeasurement< Measurement::Rotation > );
    bp::def("readCalibMeasurementMatrix3x3", &readUtCalibFileDropMeasurement< Measurement::Matrix3x3 > );
    bp::def("readCalibMeasurementMatrix3x4", &readUtCalibFileDropMeasurement< Measurement::Matrix3x4 > );
    bp::def("readCalibMeasurementMatrix4x4", &readUtCalibFileDropMeasurement< Measurement::Matrix4x4 > );
    bp::def("readCalibMeasurementVector4D", &readUtCalibFileDropMeasurement< Measurement::Vector4D > );
    bp::def("readCalibMeasurementVector8D", &readUtCalibFileDropMeasurement< Measurement::Vector8D > );
    bp::def("readCalibMeasurementPositionList", &readUtCalibFileDropMeasurement< Measurement::PositionList > );
    bp::def("readCalibMeasurementCameraIntrinsics", &readUtCalibFileDropMeasurement< Measurement::CameraIntrinsics > );
    bp::def("readCalibMeasurementPositionList2", &readUtCalibFileDropMeasurement< Measurement::PositionList2 > );
    bp::def("readCalibMeasurementPoseList", &readUtCalibFileDropMeasurement< Measurement::PoseList > );
    bp::def("readCalibMeasurementDistanceList", &readUtCalibFileDropMeasurement< Measurement::DistanceList > );

    bp::def("readCalibDistance", &readUtCalibFile< Math::Scalar< double > >);
    bp::def("readCalibPose", &readUtCalibFile< Math::Pose >);
    bp::def("readCalibErrorPose", &readUtCalibFile< Math::ErrorPose >);
    bp::def("readCalibErrorPosition", &readUtCalibFile< Math::ErrorVector< double, 3 > >);
    bp::def("readCalibPosition", &readUtCalibFile< Math::Vector3d >);
    bp::def("readCalibRotation", &readUtCalibFile< Math::Quaternion >);
    bp::def("readCalibMatrix3x3", &readUtCalibFile< Math::Matrix3x3d >);
    bp::def("readCalibMatrix3x4", &readUtCalibFile< Math::Matrix3x4d >);
    bp::def("readCalibMatrix4x4", &readUtCalibFile< Math::Matrix4x4d >);
    bp::def("readCalibVector4D", &readUtCalibFile< Math::Vector4d >);
    bp::def("readCalibVector8D", &readUtCalibFile< Math::Vector< double, 8 > >);
    bp::def("readCalibCameraIntrinsics", &readUtCalibFile< Math::CameraIntrinsics<double> >);
    bp::def("readCalibPositionList", &readUtCalibFile< std::vector< Math::Vector3d > >);
    bp::def("readCalibPositionList2", &readUtCalibFile< std::vector< Math::Vector2d > >);
    bp::def("readCalibPoseList", &readUtCalibFile< std::vector< Math::Pose > >);
    bp::def("readCalibDistanceList", &readUtCalibFile< std::vector< Math::Scalar< double > > >);



    bp::def("writeCalibMeasurementDistance", &writeUtCalibFile< Measurement::Distance >);
    bp::def("writeCalibMeasurementPose", &writeUtCalibFile< Measurement::Pose >);
    bp::def("writeCalibMeasurementErrorPose", &writeUtCalibFile< Measurement::ErrorPose >);
    bp::def("writeCalibMeasurementErrorPosition", &writeUtCalibFile< Measurement::ErrorPosition >);
    bp::def("writeCalibMeasurementPosition", &writeUtCalibFile< Measurement::Position >);
    bp::def("writeCalibMeasurementRotation", &writeUtCalibFile< Measurement::Rotation >);
    bp::def("writeCalibMeasurementMatrix3x3", &writeUtCalibFile< Measurement::Matrix3x3 >);
    bp::def("writeCalibMeasurementMatrix3x4", &writeUtCalibFile< Measurement::Matrix3x4 >);
    bp::def("writeCalibMeasurementMatrix4x4", &writeUtCalibFile< Measurement::Matrix4x4 >);
    bp::def("writeCalibMeasurementVector4D", &writeUtCalibFile< Measurement::Vector4D >);
    bp::def("writeCalibMeasurementVector8D", &writeUtCalibFile< Measurement::Vector8D >);
    bp::def("writeCalibMeasurementCameraIntrinsics", &writeUtCalibFile< Measurement::CameraIntrinsics >);
    bp::def("writeCalibMeasurementPositionList", &writeUtCalibFile< Measurement::PositionList >);
    bp::def("writeCalibMeasurementPositionList2", &writeUtCalibFile< Measurement::PositionList2 >);
    bp::def("writeCalibMeasurementPoseList", &writeUtCalibFile< Measurement::PoseList >);
    bp::def("writeCalibMeasurementDistanceList", &writeUtCalibFile< Measurement::DistanceList >);

    bp::def("writeCalibDistance", &writeUtCalibFile< Math::Scalar< double > >);
    bp::def("writeCalibPose", &writeUtCalibFile< Math::Pose >);
    bp::def("writeCalibErrorPose", &writeUtCalibFile< Math::ErrorPose >);
    bp::def("writeCalibErrorPosition", &writeUtCalibFile< Math::ErrorVector< double, 3 > >);
    bp::def("writeCalibPosition", &writeUtCalibFile< Math::Vector3d >);
    bp::def("writeCalibRotation", &writeUtCalibFile< Math::Quaternion >);
    bp::def("writeCalibMatrix3x3", &writeUtCalibFile< Math::Matrix3x3d >);
    bp::def("writeCalibMatrix3x4", &writeUtCalibFile< Math::Matrix3x4d >);
    bp::def("writeCalibMatrix4x4", &writeUtCalibFile< Math::Matrix4x4d >);
    bp::def("writeCalibVector4D", &writeUtCalibFile< Math::Vector4d >);
    bp::def("writeCalibVector8D", &writeUtCalibFile< Math::Vector< double, 8 > >);
    bp::def("writeCalibCameraIntrinsics", &writeUtCalibFile< Math::CameraIntrinsics<double> >);
    bp::def("writeCalibPositionList", &writeUtCalibFile< std::vector< Math::Vector3d > >);
    bp::def("writeCalibPositionList2", &writeUtCalibFile< std::vector< Math::Vector2d > >);
    bp::def("writeCalibPoseList", &writeUtCalibFile< std::vector< Math::Pose > >);
    bp::def("writeCalibDistanceList", &writeUtCalibFile< std::vector< Math::Scalar< double > > >);

}
