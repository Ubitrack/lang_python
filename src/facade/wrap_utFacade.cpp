#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#ifndef HAVE_BOOST_NUMPY
#include <boost/python/numpy.hpp>
#else
#include <boost/numpy.hpp>
#endif

#include <numpy/arrayobject.h>
#include <complex>

#include <boost/bind.hpp>

#include <boost/python.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/to_python_converter.hpp>

#include <utFacade/AdvancedFacade.h>
#include <utFacade/DataflowObserver.h>
#include <utComponents/ApplicationEndpointsVision.h>
#include <utComponents/ApplicationPushSink.h>
#include <utComponents/ApplicationPullSink.h>
#include <utComponents/ApplicationPushSource.h>
#include <utComponents/ApplicationPullSource.h>

#include <iostream>

using namespace Ubitrack;

namespace bp = boost::python;
#ifndef HAVE_BOOST_NUMPY
namespace bn = boost::python::numpy;
#else
namespace bn = boost::numpy;
#endif

namespace {

class PyDataflowObserver: public Facade::DataflowObserver {

public:
	PyDataflowObserver(bp::object add_notifier_, bp::object del_notifier_)
		: Facade::DataflowObserver()
		, add_notifier(add_notifier_)
		, del_notifier(del_notifier_)
	{}
	virtual ~PyDataflowObserver() {}

	virtual void notifyAddComponent( const std::string& sPatternName, const std::string& sComponentName, const Graph::UTQLSubgraph& pattern ) {
		PyGILState_STATE gstate = PyGILState_Ensure();
		add_notifier(sPatternName, sComponentName, pattern);
		PyGILState_Release( gstate );
	}

	virtual void notifyDeleteComponent( const std::string& sPatternName, const std::string& sComponentName ) {
		PyGILState_STATE gstate = PyGILState_Ensure();
		del_notifier(sPatternName, sComponentName);
		PyGILState_Release( gstate );
	}


private:
	bp::object add_notifier;
	bp::object del_notifier;
};


// decode a Python exception into a string
std::string handle_pyerror()
{
    using namespace boost::python;
    using namespace boost;

    PyObject *exc,*val,*tb;
    object formatted_list, formatted;
    PyErr_Fetch(&exc,&val,&tb);
    handle<> hexc(exc),hval(allow_null(val)),htb(allow_null(tb)); 
    object traceback(import("traceback"));
    if (!tb) {
        object format_exception_only(traceback.attr("format_exception_only"));
        formatted_list = format_exception_only(hexc,hval);
    } else {
        object format_exception(traceback.attr("format_exception"));
        formatted_list = format_exception(hexc,hval,htb);
    }
    formatted = str("\n").join(formatted_list);
    return extract<std::string>(formatted);}


template< class MT >
struct measurement_callback_wrapper_sink_t
{
	measurement_callback_wrapper_sink_t( bp::object callable ) : _callable( callable ) {}

    bool operator()(const MT& m)
    {
        PyGILState_STATE gstate = PyGILState_Ensure();
        bool ret = _callable(m);
        PyGILState_Release( gstate );
        return ret;
    }

    bp::object _callable;
};

template< class OT, class MT >
void setWrappedCallback( OT* self,  bp::object function )
{
	self->setCallback(boost::function<void (const MT&)>(
			measurement_callback_wrapper_sink_t< MT >( function ) ));
}

template< class MT >
void setWrappedCallbackFacade( Facade::AdvancedFacade* self, const std::string& name, bp::object function )
{
	self->setCallback(name, boost::function<void (const MT&)>(
			measurement_callback_wrapper_sink_t< MT >( function ) ));
}

void addDataflowObserver( Facade::AdvancedFacade* self, bp::object py_observer )
{
	PyDataflowObserver* observer = bp::extract< PyDataflowObserver* >(py_observer);
	self->addDataflowObserver(observer);
}

void removeDataflowObserver( Facade::AdvancedFacade* self, bp::object py_observer )
{
	PyDataflowObserver* observer = bp::extract< PyDataflowObserver* >(py_observer);
	self->removeDataflowObserver(observer);
}

template< class MT >
struct measurement_callback_wrapper_source_t
{
	measurement_callback_wrapper_source_t( bp::object callable ) : _callable( callable ) {}

    MT operator()(const Measurement::Timestamp t)
    {
        PyGILState_STATE gstate = PyGILState_Ensure();
		MT result;
		try {
			bp::object ob = _callable(t);
			result = bp::extract<MT>(ob);
		}
		catch (const bp::error_already_set&) {
	        if (PyErr_Occurred()) {
	            UBITRACK_THROW(handle_pyerror()); 
	        }
	        bp::handle_exception();
	        PyErr_Clear();
		}
        PyGILState_Release( gstate );
        return result;
    }

    bp::object _callable;
};

template< class OT, class MT >
void setWrappedSourceCallback( OT* self,  bp::object function )
{
	self->setCallback(boost::function< MT (const Ubitrack::Measurement::Timestamp)>(
			measurement_callback_wrapper_source_t< MT >( function ) ));
}

// template< class MT >
// void setWrappedSourceCallbackFacade( Facade::AdvancedFacade* self, const std::string& name, bp::object function )
// {
// 	self->setCallback(name, boost::function< MT (const Ubitrack::Measurement::Timestamp)>(
// 			measurement_callback_wrapper_source_t< MT >( function ) ));
// }
//

template< class ComponentType, class EventType >
void expose_pushsink_for(const std::string& event_name)
{
	std::string pushsink_name("ApplicationPushSink");
	pushsink_name.append(event_name);
	bp::class_< ComponentType, boost::shared_ptr< ComponentType >, bp::bases< Dataflow::Component >, boost::noncopyable>(pushsink_name.c_str(), bp::no_init)
		.def("setCallback", &setWrappedCallback< ComponentType, EventType >)
		;
}

template< class ComponentType, class EventType >
void expose_pullsource_for(const std::string& event_name)
{
	std::string pullsource_name("ApplicationPullSource");
	pullsource_name.append(event_name);
	bp::class_< ComponentType, boost::shared_ptr< ComponentType >, bp::bases< Dataflow::Component >, boost::noncopyable>(pullsource_name.c_str(), bp::no_init)
		.def("setCallback", &setWrappedSourceCallback< ComponentType, EventType >)
		;
}

template< class EventType >
void expose_pullsink_for(const std::string& event_name)
{
	std::string pullsink_name("ApplicationPullSink");
	pullsink_name.append(event_name);
	bp::class_< Components::ApplicationPullSink< EventType >, boost::shared_ptr< Components::ApplicationPullSink< EventType > >, bp::bases< Dataflow::Component >, boost::noncopyable>(pullsink_name.c_str(), bp::no_init)
		.def("get", &Components::ApplicationPullSink< EventType >::get)
		;
}

template< class ComponentType >
void expose_pushsource_for(const std::string& event_name)
{
	std::string pushsource_name("ApplicationPushSource");
	pushsource_name.append(event_name);
	bp::class_< ComponentType, boost::shared_ptr< ComponentType >, bp::bases< Dataflow::Component >, boost::noncopyable>(pushsource_name.c_str(), bp::no_init)
		.def("send", &ComponentType::send)
		;
}


}

BOOST_PYTHON_MODULE(_utfacade)
{
	// push sinks
	expose_pushsink_for< Components::ApplicationPushSinkButton, Measurement::Button >("Button");
	expose_pushsink_for< Components::ApplicationPushSinkDistance, Measurement::Distance >("Distance");
	expose_pushsink_for< Components::ApplicationPushSinkPose, Measurement::Pose >("Pose");
	expose_pushsink_for< Components::ApplicationPushSinkErrorPose, Measurement::ErrorPose >("ErrorPose");
	expose_pushsink_for< Components::ApplicationPushSinkPosition, Measurement::Position >("Position");
	expose_pushsink_for< Components::ApplicationPushSinkPosition2D, Measurement::Position2D >("Position2D");
	expose_pushsink_for< Components::ApplicationPushSinkRotation, Measurement::Rotation >("Rotation");
	expose_pushsink_for< Components::ApplicationPushSinkMatrix4x4, Measurement::Matrix4x4 >("Matrix4x4");
	expose_pushsink_for< Components::ApplicationPushSinkMatrix3x3, Measurement::Matrix3x3 >("Matrix3x3");
	expose_pushsink_for< Components::ApplicationPushSinkMatrix3x4, Measurement::Matrix3x4 >("Matrix3x4");

	expose_pushsink_for< Components::ApplicationPushSinkPositionList, Measurement::PositionList >("PositionList");
	expose_pushsink_for< Components::ApplicationPushSinkPositionList2, Measurement::PositionList2 >("PositionList2");
	//expose_pushsink_for< Components::ApplicationPushSinkErrorPositionList, Measurement::ErrorPositionList >("ErrorPositionList");
	//expose_pushsink_for< Components::ApplicationPushSinkErrorPositionList2, Measurement::ErrorPositionList2 >("ErrorPositionList2");

	expose_pushsink_for< Components::ApplicationPushSink< Measurement::ImageMeasurement >, Measurement::ImageMeasurement >("VisionImage");

	// push sources
	expose_pushsource_for< Components::ApplicationPushSourceButton >("Button");
	expose_pushsource_for< Components::ApplicationPushSourceDistance >("Distance");
	expose_pushsource_for< Components::ApplicationPushSourcePose >("Pose");
	expose_pushsource_for< Components::ApplicationPushSourceErrorPose>("ErrorPose");
	expose_pushsource_for< Components::ApplicationPushSourcePosition >("Position");
	expose_pushsource_for< Components::ApplicationPushSourcePosition2D >("Position2D");
	expose_pushsource_for< Components::ApplicationPushSourceRotation >("Rotation");
	expose_pushsource_for< Components::ApplicationPushSourceMatrix4x4 >("Matrix4x4");
	expose_pushsource_for< Components::ApplicationPushSourceMatrix3x3 >("Matrix3x3");
	expose_pushsource_for< Components::ApplicationPushSourceMatrix3x4 >("Matrix3x4");

	expose_pushsource_for< Components::ApplicationPushSourcePositionList >("PositionList");
	expose_pushsource_for< Components::ApplicationPushSourcePositionList2 >("Position2DList");
	expose_pushsource_for< Components::ApplicationPushSourceErrorPositionList >("ErrorPositionList");
	expose_pushsource_for< Components::ApplicationPushSourceErrorPositionList2 >("ErrorPositionList2");

	//expose_pushsource_for< Components::ApplicationPushSource< Measurement::ImageMeasurement > >("VisionImage");


	// pull sinks
	expose_pullsink_for< Measurement::Button >("Button");
	expose_pullsink_for< Measurement::Distance >("Distance");
	expose_pullsink_for< Measurement::Pose >("Pose");
	expose_pullsink_for< Measurement::ErrorPose >("ErrorPose");
	expose_pullsink_for< Measurement::Position2D >("Position2D");
	expose_pullsink_for< Measurement::Position >("Position");
	expose_pullsink_for< Measurement::ErrorPosition >("ErrorPosition");
	expose_pullsink_for< Measurement::Rotation >("Rotation");
	expose_pullsink_for< Measurement::Matrix4x4 >("Matrix4x4");
	expose_pullsink_for< Measurement::Matrix3x3 >("Matrix3x3");
	expose_pullsink_for< Measurement::Vector4D >("Vector4D");

	expose_pullsink_for< Measurement::PositionList >("PositionList");
	expose_pullsink_for< Measurement::PositionList2 >("PositionList2");
	expose_pullsink_for< Measurement::ErrorPositionList >("ErrorPositionList");
	expose_pullsink_for< Measurement::ErrorPositionList2 >("ErrorPositionList2");

	expose_pullsink_for< Measurement::ImageMeasurement >("VisionImage");

	// pull sources
	expose_pullsource_for< Components::ApplicationPullSourceButton, Measurement::Button >("Button");
	expose_pullsource_for< Components::ApplicationPullSourceDistance, Measurement::Distance >("Distance");
	expose_pullsource_for< Components::ApplicationPullSourcePose, Measurement::Pose >("Pose");
	expose_pullsource_for< Components::ApplicationPullSourceErrorPose, Measurement::ErrorPose >("ErrorPose");
	expose_pullsource_for< Components::ApplicationPullSourcePosition, Measurement::Position >("Position");
	expose_pullsource_for< Components::ApplicationPullSourcePosition2D, Measurement::Position2D >("Position2D");
	expose_pullsource_for< Components::ApplicationPullSourceRotation, Measurement::Rotation >("Rotation");
	expose_pullsource_for< Components::ApplicationPullSourceMatrix4x4, Measurement::Matrix4x4 >("Matrix4x4");
	expose_pullsource_for< Components::ApplicationPullSourceMatrix3x3, Measurement::Matrix3x3 >("Matrix3x3");
	expose_pullsource_for< Components::ApplicationPullSourceMatrix3x4, Measurement::Matrix3x4 >("Matrix3x4");

	expose_pullsource_for< Components::ApplicationPullSourcePositionList, Measurement::PositionList >("PositionList");
	expose_pullsource_for< Components::ApplicationPullSourcePositionList2, Measurement::PositionList2 >("PositionList2");
	expose_pullsource_for< Components::ApplicationPullSourceErrorPositionList, Measurement::ErrorPositionList >("ErrorPositionList");
	expose_pullsource_for< Components::ApplicationPullSourceErrorPositionList2, Measurement::ErrorPositionList2 >("ErrorPositionList2");

	//expose_pullsource_for< Components::ApplicationPullSource< Measurement::ImageMeasurement >, Measurement::ImageMeasurement >("VisionImage");

	bp::class_< PyDataflowObserver, boost::shared_ptr< PyDataflowObserver >, boost::noncopyable>("DataflowObserver", bp::init< bp::object, bp::object>());


	bp::class_< Facade::AdvancedFacade, boost::shared_ptr< Facade::AdvancedFacade >, boost::noncopyable>("AdvancedFacade", bp::init< bp::optional< const std::string& > >())
		.def(bp::init<bool, const std::string&>())
		.def("loadDataflow", (void (Facade::AdvancedFacade::*)(const std::string&, bool))&Facade::AdvancedFacade::loadDataflow)
		.def("loadDataflow", (void (Facade::AdvancedFacade::*)(std::istream&, bool))&Facade::AdvancedFacade::loadDataflow)
		.def("clearDataflow", &Facade::AdvancedFacade::clearDataflow)
		.def("startDataflow", &Facade::AdvancedFacade::startDataflow)
		.def("stopDataflow", &Facade::AdvancedFacade::stopDataflow)
		.def("killEverything", &Facade::AdvancedFacade::killEverything)

		// all components .. need a better way do do this .. meta-meta programming needed - havent looking into boost::mpl yet ;)

		// push sinks
		.def("getApplicationPushSinkButton", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSinkButton >)
		.def("getApplicationPushSinkDistance", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSinkDistance >)
		.def("getApplicationPushSinkPose", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSinkPose >)
		.def("getApplicationPushSinkErrorPose", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSinkErrorPose >)
		.def("getApplicationPushSinkPosition", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSinkPosition >)
		.def("getApplicationPushSinkPosition2D", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSinkPosition2D >)
		.def("getApplicationPushSinkRotation", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSinkRotation >)
		.def("getApplicationPushSinkMatrix4x4", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSinkMatrix4x4 >)
		.def("getApplicationPushSinkMatrix3x3", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSinkMatrix3x3 >)
		.def("getApplicationPushSinkMatrix3x4", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSinkMatrix3x4 >)

		.def("getApplicationPushSinkPositionList", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSinkPositionList >)
		.def("getApplicationPushSinkPositionList2", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSinkPositionList2 >)
		.def("getApplicationPushSinkErrorPositionList", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSinkErrorPositionList >)
		.def("getApplicationPushSinkErrorPositionList2", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSinkErrorPositionList2 >)

		.def("getApplicationPushSinkVisionImage", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSink< Measurement::ImageMeasurement > >)


		// push sources
		.def("getApplicationPushSourceButton", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSourceButton >)
		.def("getApplicationPushSourceDistance", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSourceDistance >)
		.def("getApplicationPushSourcePose", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSourcePose >)
		.def("getApplicationPushSourceErrorPose", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSourceErrorPose >)
		.def("getApplicationPushSourcePosition", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSourcePosition >)
		.def("getApplicationPushSourcePosition2D", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSourcePosition2D >)
		.def("getApplicationPushSourceRotation", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSourceRotation >)
		.def("getApplicationPushSourceMatrix4x4", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSourceMatrix4x4 >)
		.def("getApplicationPushSourceMatrix3x4", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSourceMatrix3x4 >)
		.def("getApplicationPushSourceMatrix3x3", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSourceMatrix3x3 >)

		.def("getApplicationPushSourcePositionList", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSourcePositionList >)
		.def("getApplicationPushSourcePosition2DList", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSourcePositionList2 >)
		.def("getApplicationPushSourceErrorPositionList", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSourceErrorPositionList >)
		.def("getApplicationPushSourceErrorPositionList2", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSourceErrorPositionList2 >)

		//.def("getApplicationPushSourceVisionImage", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSource< Measurement::ImageMeasurement > >)



		// pull sinks
		.def("getApplicationPullSinkButton", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Measurement::Button > >)
		.def("getApplicationPullSinkDistance", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Measurement::Distance > >)
		.def("getApplicationPullSinkPose", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Measurement::Pose > >)
		.def("getApplicationPullSinkErrorPose", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Measurement::ErrorPose > >)
		.def("getApplicationPullSinkPosition", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Measurement::Position > >)
		.def("getApplicationPullSinkPosition2D", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Measurement::Position2D > >)
		.def("getApplicationPullSinkRotation", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Measurement::Rotation > >)
		.def("getApplicationPullSinkMatrix4x4", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Measurement::Matrix4x4 > >)
		.def("getApplicationPullSinkMatrix3x3", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Measurement::Matrix3x3 > >)
		.def("getApplicationPullSinkVector4D", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Measurement::Vector4D > >)

		.def("getApplicationPullSinkPositionList", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Measurement::PositionList > >)
		.def("getApplicationPullSinkPositionList2", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Measurement::PositionList2 > >)
		.def("getApplicationPullSinkErrorPositionList", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Measurement::ErrorPositionList > >)
		.def("getApplicationPullSinkErrorPositionList2", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Measurement::ErrorPositionList2 > >)


		.def("getApplicationPullSinkVisionImage", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Measurement::ImageMeasurement > >)

		// pull sources
		.def("getApplicationPullSourceButton", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSourceButton >)
		.def("getApplicationPullSourceDistance", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSourceDistance >)
		.def("getApplicationPullSourcePose", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSourcePose >)
		.def("getApplicationPullSourceErrorPose", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSourceErrorPose >)
		.def("getApplicationPullSourcePosition", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSourcePosition >)
		.def("getApplicationPullSourcePosition2D", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSourcePosition2D >)
		.def("getApplicationPullSourceRotation", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSourceRotation >)
		.def("getApplicationPullSourceMatrix4x4", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSourceMatrix4x4 >)
		.def("getApplicationPullSourceMatrix3x3", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSourceMatrix3x3 >)
		.def("getApplicationPullSourceMatrix3x4", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSourceMatrix3x4 >)

		.def("getApplicationPullSourcePositionList", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSourcePositionList >)
		.def("getApplicationPullSourcePositionList2", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSourcePositionList2 >)
		.def("getApplicationPullSourceErrorPositionList", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSourceErrorPositionList >)
		.def("getApplicationPullSourceErrorPositionList2", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSourceErrorPositionList2 >)

		//.def("getApplicationPullSourceVisionImage", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSink< Measurement::ImageMeasurement > >)




		// push sinks
		.def("setCallbackButton", &setWrappedCallbackFacade< Measurement::Button >)
		.def("setCallbackDistance", &setWrappedCallbackFacade< Measurement::Distance >)
		.def("setCallbackPose", &setWrappedCallbackFacade< Measurement::Pose >)
		.def("setCallbackErrorPose", &setWrappedCallbackFacade< Measurement::ErrorPose >)
		.def("setCallbackPosition", &setWrappedCallbackFacade< Measurement::Position >)
		.def("setCallbackPosition2D", &setWrappedCallbackFacade< Measurement::Position2D >)
		.def("setCallbackRotation", &setWrappedCallbackFacade< Measurement::Rotation >)
		.def("setCallbackMatrix3x3", &setWrappedCallbackFacade< Measurement::Matrix4x4 >)
		.def("setCallbackMatrix4x4", &setWrappedCallbackFacade< Measurement::Matrix3x3 >)

		.def("setCallbackPositionList", &setWrappedCallbackFacade< Measurement::PositionList >)
		.def("setCallbackPositionList2", &setWrappedCallbackFacade< Measurement::PositionList2 >)
//		.def("setCallbackErrorPositionList", &setWrappedCallbackFacade< Measurement::ErrorPositionList >)
//		.def("setCallbackErrorPositionList2", &setWrappedCallbackFacade< Measurement::ErrorPositionList2 >)

		.def("setCallbackVisionImage", &setWrappedCallbackFacade< Measurement::ImageMeasurement >)

		// pull source (not supported by AdvancedFacade ..)
		// .def("setSourceCallbackButton", &setWrappedSourceCallbackFacade< Measurement::Button >)
		// .def("setSourceCallbackPose", &setWrappedSourceCallbackFacade< Measurement::Pose >)
		// .def("setSourceCallbackErrorPose", &setWrappedSourceCallbackFacade< Measurement::ErrorPose >)
		// .def("setSourceCallbackPosition", &setWrappedSourceCallbackFacade< Measurement::Position >)
		// .def("setSourceCallbackPosition2D", &setWrappedSourceCallbackFacade< Measurement::Position2D >)
		// .def("setSourceCallbackRotation", &setWrappedSourceCallbackFacade< Measurement::Rotation >)
		// .def("setSourceCallbackMatrix3x3", &setWrappedSourceCallbackFacade< Measurement::Matrix4x4 >)
		// .def("setSourceCallbackMatrix4x4", &setWrappedSourceCallbackFacade< Measurement::Matrix3x3 >)
		//
		// .def("setSourceCallbackPositionList", &setWrappedSourceCallbackFacade< Measurement::PositionList >)
		// .def("setSourceCallbackPositionList2", &setWrappedSourceCallbackFacade< Measurement::PositionList2 >)
		//.def("setSourceCallbackErrorPositionList", &setWrappedSourceCallbackFacade< Measurement::ErrorPositionList >)
		//.def("setSourceCallbackErrorPositionList2", &setWrappedSourceCallbackFacade< Measurement::ErrorPositionList2 >)

		//.def("setSourceCallbackVisionImage", &setWrappedSourceCallbackFacade< Measurement::ImageMeasurement >)



		.def("addDataflowObserver", &addDataflowObserver)
		.def("removeDataflowObserver", &removeDataflowObserver)
		;


}
