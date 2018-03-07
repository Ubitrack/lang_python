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

#include <utFacade/AdvancedFacade.h>
#include <utFacade/DataflowObserver.h>
#include <utComponents/ApplicationEndpointsVision.h>
#include <utComponents/ApplicationPushSink.h>
#include <utComponents/ApplicationPullSink.h>
#include <utComponents/ApplicationPushSource.h>
#include <utComponents/ApplicationPullSource.h>

#include <iostream>
#include <strstream>

class PyDataflowObserver: public Ubitrack::Facade::DataflowObserver {

public:
	using Ubitrack::Facade::DataflowObserver::DataflowObserver;

	PyDataflowObserver(py::object add_notifier_, py::object del_notifier_)
		: Ubitrack::Facade::DataflowObserver()
		, add_notifier(add_notifier_)
		, del_notifier(del_notifier_)
	{}
	virtual ~PyDataflowObserver() {}

	virtual void notifyAddComponent( const std::string& sPatternName, const std::string& sComponentName, const Ubitrack::Graph::UTQLSubgraph& pattern ) {
		py::gil_scoped_acquire acquire;
		if (!add_notifier.is_none())
			add_notifier(sPatternName, sComponentName, pattern);
	}

	virtual void notifyDeleteComponent( const std::string& sPatternName, const std::string& sComponentName ) {
		py::gil_scoped_acquire acquire;
		if (!del_notifier.is_none())
			del_notifier(sPatternName, sComponentName);
	}


private:
	py::object add_notifier;
	py::object del_notifier;
};



// decode a Python exception and traceback into a string
std::string handle_pyerror()
{
	py::object type, value, trace;
    py::list formatted_list;

    PyErr_Fetch(&type.ptr(), &value.ptr(), &trace.ptr());

    py::object traceback(py::module::import("traceback"));

    if (!trace.is_none()) {
        py::object format_exception_only(traceback.attr("format_exception_only"));
        formatted_list = format_exception_only(type,value);
    } else {
        py::object format_exception(traceback.attr("format_exception"));
        formatted_list = format_exception(type,value,trace);
    }

    std::strstream ret;
    for (auto item : formatted_list) {
    	ret << item.cast<std::string>();
    	ret << "\n";
    }
    return std::string(ret.str());
}


template< class MT >
struct measurement_callback_wrapper_sink_t
{
	measurement_callback_wrapper_sink_t( py::object callable ) : _callable( callable ) {}

    bool operator()(const MT& m)
    {
		py::gil_scoped_acquire acquire;
        py::object ret = _callable(m);
        return ret.cast<bool>();
    }

    py::object _callable;
};

template< class OT, class MT >
void setWrappedCallback( OT* self,  py::object function )
{
	self->setCallback(boost::function<void (const MT&)>(
			measurement_callback_wrapper_sink_t< MT >( function ) ));
}

template< class MT >
void setWrappedCallbackFacade( Ubitrack::Facade::AdvancedFacade* self, const std::string& name, py::object function )
{
	self->setCallback(name, boost::function<void (const MT&)>(
			measurement_callback_wrapper_sink_t< MT >( function ) ));
}

void addDataflowObserver( Ubitrack::Facade::AdvancedFacade* self, py::object py_observer )
{
	PyDataflowObserver* observer = py_observer.cast< PyDataflowObserver* >();
	self->addDataflowObserver(observer);
}

void removeDataflowObserver( Ubitrack::Facade::AdvancedFacade* self, py::object py_observer )
{
	PyDataflowObserver* observer = py_observer.cast< PyDataflowObserver* >();
	self->removeDataflowObserver(observer);
}

template< class MT >
struct measurement_callback_wrapper_source_t
{
	measurement_callback_wrapper_source_t( py::object callable ) : _callable( callable ) {}

    MT operator()(const Ubitrack::Measurement::Timestamp t)
    {
		py::gil_scoped_acquire acquire;
		MT result;
		try {
			py::object ob = _callable(t);
			result = ob.cast<MT>();
		}
		catch (const py::error_already_set& e) {
	        if (PyErr_Occurred()) {
	            UBITRACK_THROW(handle_pyerror()); 
	        }

	        // how to handle a situation where ithe error did not originate from python
	        // bp::handle_exception();
	        PyErr_Clear();
		}
        return result;
    }

    py::object _callable;
};

template< class OT, class MT >
void setWrappedSourceCallback( OT* self,  py::object function )
{
	self->setCallback(boost::function< MT (const Ubitrack::Measurement::Timestamp)>(
			measurement_callback_wrapper_source_t< MT >( function ) ));
}

// template< class MT >
// void setWrappedSourceCallbackFacade( Ubitrack::Facade::AdvancedFacade* self, const std::string& name, py::object function )
// {
// 	self->setCallback(name, boost::function< MT (const Ubitrack::Measurement::Timestamp)>(
// 			measurement_callback_wrapper_source_t< MT >( function ) ));
// }
//


template< class ComponentType, class EventType >
void expose_pushsink_for(py::module &m, const std::string& event_name)
{
	std::string pushsink_name("ApplicationPushSink");
	pushsink_name.append(event_name);
	py::class_< ComponentType, Ubitrack::Dataflow::Component, boost::shared_ptr< ComponentType >>(m, pushsink_name.c_str())
		.def("setCallback", &setWrappedCallback< ComponentType, EventType >)
		;
}

template< class ComponentType, class EventType >
void expose_pullsource_for(py::module &m, const std::string& event_name)
{
	std::string pullsource_name("ApplicationPullSource");
	pullsource_name.append(event_name);
	py::class_< ComponentType, Ubitrack::Dataflow::Component, boost::shared_ptr< ComponentType >>(m, pullsource_name.c_str())
		.def("setCallback", &setWrappedSourceCallback< ComponentType, EventType >)
		;
}

template< class EventType >
void expose_pullsink_for(py::module &m, const std::string& event_name)
{
	std::string pullsink_name("ApplicationPullSink");
	pullsink_name.append(event_name);
	py::class_< Ubitrack::Components::ApplicationPullSink< EventType >, Ubitrack::Dataflow::Component, boost::shared_ptr< Ubitrack::Components::ApplicationPullSink< EventType > >>(m, pullsink_name.c_str())
		.def("get", &Ubitrack::Components::ApplicationPullSink< EventType >::get)
		;
}

template< class ComponentType >
void expose_pushsource_for(py::module &m, const std::string& event_name)
{
	std::string pushsource_name("ApplicationPushSource");
	pushsource_name.append(event_name);
	py::class_< ComponentType, Ubitrack::Dataflow::Component, boost::shared_ptr< ComponentType >>(m, pushsource_name.c_str())
		.def("send", &ComponentType::send)
		;
}


void bind_utFacadeMain(py::module& m)
{
	using namespace Ubitrack;

	// push sinks
	expose_pushsink_for< Components::ApplicationPushSinkButton, Ubitrack::Measurement::Button >(m, "Button");
	expose_pushsink_for< Components::ApplicationPushSinkDistance, Ubitrack::Measurement::Distance >(m, "Distance");
	expose_pushsink_for< Components::ApplicationPushSinkPose, Ubitrack::Measurement::Pose >(m, "Pose");
	expose_pushsink_for< Components::ApplicationPushSinkErrorPose, Ubitrack::Measurement::ErrorPose >(m, "ErrorPose");
	expose_pushsink_for< Components::ApplicationPushSinkPosition, Ubitrack::Measurement::Position >(m, "Position");
	expose_pushsink_for< Components::ApplicationPushSinkPosition2D, Ubitrack::Measurement::Position2D >(m, "Position2D");
	expose_pushsink_for< Components::ApplicationPushSinkRotation, Ubitrack::Measurement::Rotation >(m, "Rotation");
	expose_pushsink_for< Components::ApplicationPushSinkMatrix4x4, Ubitrack::Measurement::Matrix4x4 >(m, "Matrix4x4");
	expose_pushsink_for< Components::ApplicationPushSinkMatrix3x3, Ubitrack::Measurement::Matrix3x3 >(m, "Matrix3x3");
	expose_pushsink_for< Components::ApplicationPushSinkMatrix3x4, Ubitrack::Measurement::Matrix3x4 >(m, "Matrix3x4");

	expose_pushsink_for< Components::ApplicationPushSinkPositionList, Ubitrack::Measurement::PositionList >(m, "PositionList");
	expose_pushsink_for< Components::ApplicationPushSinkPositionList2, Ubitrack::Measurement::PositionList2 >(m, "PositionList2");
	//expose_pushsink_for< Components::ApplicationPushSinkErrorPositionList, Ubitrack::Measurement::ErrorPositionList >(m, "ErrorPositionList");
	//expose_pushsink_for< Components::ApplicationPushSinkErrorPositionList2, Ubitrack::Measurement::ErrorPositionList2 >(m, "ErrorPositionList2");

	// enable later
	// expose_pushsink_for< Components::ApplicationPushSink< Ubitrack::Measurement::ImageMeasurement >, Ubitrack::Measurement::ImageMeasurement >(m, "VisionImage");

	// push sources
	expose_pushsource_for< Components::ApplicationPushSourceButton >(m, "Button");
	expose_pushsource_for< Components::ApplicationPushSourceDistance >(m, "Distance");
	expose_pushsource_for< Components::ApplicationPushSourcePose >(m, "Pose");
	expose_pushsource_for< Components::ApplicationPushSourceErrorPose>(m, "ErrorPose");
	expose_pushsource_for< Components::ApplicationPushSourcePosition >(m, "Position");
	expose_pushsource_for< Components::ApplicationPushSourcePosition2D >(m, "Position2D");
	expose_pushsource_for< Components::ApplicationPushSourceRotation >(m, "Rotation");
	expose_pushsource_for< Components::ApplicationPushSourceMatrix4x4 >(m, "Matrix4x4");
	expose_pushsource_for< Components::ApplicationPushSourceMatrix3x3 >(m, "Matrix3x3");
	expose_pushsource_for< Components::ApplicationPushSourceMatrix3x4 >(m, "Matrix3x4");

	expose_pushsource_for< Components::ApplicationPushSourcePositionList >(m, "PositionList");
	expose_pushsource_for< Components::ApplicationPushSourcePositionList2 >(m, "Position2DList");
	expose_pushsource_for< Components::ApplicationPushSourceErrorPositionList >(m, "ErrorPositionList");
	expose_pushsource_for< Components::ApplicationPushSourceErrorPositionList2 >(m, "ErrorPositionList2");

	//expose_pushsource_for< Components::ApplicationPushSource< Ubitrack::Measurement::ImageMeasurement > >(m, "VisionImage");


	// pull sinks
	expose_pullsink_for< Ubitrack::Measurement::Button >(m, "Button");
	expose_pullsink_for< Ubitrack::Measurement::Distance >(m, "Distance");
	expose_pullsink_for< Ubitrack::Measurement::Pose >(m, "Pose");
	expose_pullsink_for< Ubitrack::Measurement::ErrorPose >(m, "ErrorPose");
	expose_pullsink_for< Ubitrack::Measurement::Position2D >(m, "Position2D");
	expose_pullsink_for< Ubitrack::Measurement::Position >(m, "Position");
	expose_pullsink_for< Ubitrack::Measurement::ErrorPosition >(m, "ErrorPosition");
	expose_pullsink_for< Ubitrack::Measurement::Rotation >(m, "Rotation");
	expose_pullsink_for< Ubitrack::Measurement::Matrix4x4 >(m, "Matrix4x4");
	expose_pullsink_for< Ubitrack::Measurement::Matrix3x3 >(m, "Matrix3x3");
	expose_pullsink_for< Ubitrack::Measurement::Vector4D >(m, "Vector4D");

	expose_pullsink_for< Ubitrack::Measurement::PositionList >(m, "PositionList");
	expose_pullsink_for< Ubitrack::Measurement::PositionList2 >(m, "PositionList2");
	expose_pullsink_for< Ubitrack::Measurement::ErrorPositionList >(m, "ErrorPositionList");
	expose_pullsink_for< Ubitrack::Measurement::ErrorPositionList2 >(m, "ErrorPositionList2");

	expose_pullsink_for< Ubitrack::Measurement::ImageMeasurement >(m, "VisionImage");

	// pull sources
	expose_pullsource_for< Components::ApplicationPullSourceButton, Ubitrack::Measurement::Button >(m, "Button");
	expose_pullsource_for< Components::ApplicationPullSourceDistance, Ubitrack::Measurement::Distance >(m, "Distance");
	expose_pullsource_for< Components::ApplicationPullSourcePose, Ubitrack::Measurement::Pose >(m, "Pose");
	expose_pullsource_for< Components::ApplicationPullSourceErrorPose, Ubitrack::Measurement::ErrorPose >(m, "ErrorPose");
	expose_pullsource_for< Components::ApplicationPullSourcePosition, Ubitrack::Measurement::Position >(m, "Position");
	expose_pullsource_for< Components::ApplicationPullSourcePosition2D, Ubitrack::Measurement::Position2D >(m, "Position2D");
	expose_pullsource_for< Components::ApplicationPullSourceRotation, Ubitrack::Measurement::Rotation >(m, "Rotation");
	expose_pullsource_for< Components::ApplicationPullSourceMatrix4x4, Ubitrack::Measurement::Matrix4x4 >(m, "Matrix4x4");
	expose_pullsource_for< Components::ApplicationPullSourceMatrix3x3, Ubitrack::Measurement::Matrix3x3 >(m, "Matrix3x3");
	expose_pullsource_for< Components::ApplicationPullSourceMatrix3x4, Ubitrack::Measurement::Matrix3x4 >(m, "Matrix3x4");

	expose_pullsource_for< Components::ApplicationPullSourcePositionList, Ubitrack::Measurement::PositionList >(m, "PositionList");
	expose_pullsource_for< Components::ApplicationPullSourcePositionList2, Ubitrack::Measurement::PositionList2 >(m, "PositionList2");
	expose_pullsource_for< Components::ApplicationPullSourceErrorPositionList, Ubitrack::Measurement::ErrorPositionList >(m, "ErrorPositionList");
	expose_pullsource_for< Components::ApplicationPullSourceErrorPositionList2, Ubitrack::Measurement::ErrorPositionList2 >(m, "ErrorPositionList2");

	//expose_pullsource_for< Components::ApplicationPullSource< Ubitrack::Measurement::ImageMeasurement >, Ubitrack::Measurement::ImageMeasurement >(m, "VisionImage");

	py::class_< PyDataflowObserver, boost::shared_ptr< PyDataflowObserver >>(m, "DataflowObserver")
		.def(py::init< py::object, py::object>());


	py::class_< Facade::AdvancedFacade, boost::shared_ptr< Facade::AdvancedFacade >>(m, "AdvancedFacade")
		.def(py::init<const std::string&>(), py::arg("sComponentPath") = "")
		.def(py::init<bool, const std::string&>(), py::arg("dropEvents") = false, py::arg("sComponentPath") = "")
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

		// .def("getApplicationPushSinkVisionImage", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSink< Ubitrack::Measurement::ImageMeasurement > >)


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

		//.def("getApplicationPushSourceVisionImage", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSource< Ubitrack::Measurement::ImageMeasurement > >)



		// pull sinks
		.def("getApplicationPullSinkButton", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Ubitrack::Measurement::Button > >)
		.def("getApplicationPullSinkDistance", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Ubitrack::Measurement::Distance > >)
		.def("getApplicationPullSinkPose", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Ubitrack::Measurement::Pose > >)
		.def("getApplicationPullSinkErrorPose", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Ubitrack::Measurement::ErrorPose > >)
		.def("getApplicationPullSinkPosition", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Ubitrack::Measurement::Position > >)
		.def("getApplicationPullSinkPosition2D", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Ubitrack::Measurement::Position2D > >)
		.def("getApplicationPullSinkRotation", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Ubitrack::Measurement::Rotation > >)
		.def("getApplicationPullSinkMatrix4x4", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Ubitrack::Measurement::Matrix4x4 > >)
		.def("getApplicationPullSinkMatrix3x3", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Ubitrack::Measurement::Matrix3x3 > >)
		.def("getApplicationPullSinkVector4D", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Ubitrack::Measurement::Vector4D > >)

		.def("getApplicationPullSinkPositionList", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Ubitrack::Measurement::PositionList > >)
		.def("getApplicationPullSinkPositionList2", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Ubitrack::Measurement::PositionList2 > >)
		.def("getApplicationPullSinkErrorPositionList", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Ubitrack::Measurement::ErrorPositionList > >)
		.def("getApplicationPullSinkErrorPositionList2", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Ubitrack::Measurement::ErrorPositionList2 > >)


		// .def("getApplicationPullSinkVisionImage", &Facade::AdvancedFacade::componentByName< Components::ApplicationPullSink< Ubitrack::Measurement::ImageMeasurement > >)

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

		//.def("getApplicationPullSourceVisionImage", &Facade::AdvancedFacade::componentByName< Components::ApplicationPushSink< Ubitrack::Measurement::ImageMeasurement > >)




		// push sinks
		.def("setCallbackButton", &setWrappedCallbackFacade< Ubitrack::Measurement::Button >)
		.def("setCallbackDistance", &setWrappedCallbackFacade< Ubitrack::Measurement::Distance >)
		.def("setCallbackPose", &setWrappedCallbackFacade< Ubitrack::Measurement::Pose >)
		.def("setCallbackErrorPose", &setWrappedCallbackFacade< Ubitrack::Measurement::ErrorPose >)
		.def("setCallbackPosition", &setWrappedCallbackFacade< Ubitrack::Measurement::Position >)
		.def("setCallbackPosition2D", &setWrappedCallbackFacade< Ubitrack::Measurement::Position2D >)
		.def("setCallbackRotation", &setWrappedCallbackFacade< Ubitrack::Measurement::Rotation >)
		.def("setCallbackMatrix3x3", &setWrappedCallbackFacade< Ubitrack::Measurement::Matrix4x4 >)
		.def("setCallbackMatrix4x4", &setWrappedCallbackFacade< Ubitrack::Measurement::Matrix3x3 >)

		.def("setCallbackPositionList", &setWrappedCallbackFacade< Ubitrack::Measurement::PositionList >)
		.def("setCallbackPositionList2", &setWrappedCallbackFacade< Ubitrack::Measurement::PositionList2 >)
//		.def("setCallbackErrorPositionList", &setWrappedCallbackFacade< Ubitrack::Measurement::ErrorPositionList >)
//		.def("setCallbackErrorPositionList2", &setWrappedCallbackFacade< Ubitrack::Measurement::ErrorPositionList2 >)

		// .def("setCallbackVisionImage", &setWrappedCallbackFacade< Ubitrack::Measurement::ImageMeasurement >)

		// pull source (not supported by AdvancedFacade ..)
		// .def("setSourceCallbackButton", &setWrappedSourceCallbackFacade< Ubitrack::Measurement::Button >)
		// .def("setSourceCallbackPose", &setWrappedSourceCallbackFacade< Ubitrack::Measurement::Pose >)
		// .def("setSourceCallbackErrorPose", &setWrappedSourceCallbackFacade< Ubitrack::Measurement::ErrorPose >)
		// .def("setSourceCallbackPosition", &setWrappedSourceCallbackFacade< Ubitrack::Measurement::Position >)
		// .def("setSourceCallbackPosition2D", &setWrappedSourceCallbackFacade< Ubitrack::Measurement::Position2D >)
		// .def("setSourceCallbackRotation", &setWrappedSourceCallbackFacade< Ubitrack::Measurement::Rotation >)
		// .def("setSourceCallbackMatrix3x3", &setWrappedSourceCallbackFacade< Ubitrack::Measurement::Matrix4x4 >)
		// .def("setSourceCallbackMatrix4x4", &setWrappedSourceCallbackFacade< Ubitrack::Measurement::Matrix3x3 >)
		//
		// .def("setSourceCallbackPositionList", &setWrappedSourceCallbackFacade< Ubitrack::Measurement::PositionList >)
		// .def("setSourceCallbackPositionList2", &setWrappedSourceCallbackFacade< Ubitrack::Measurement::PositionList2 >)
		//.def("setSourceCallbackErrorPositionList", &setWrappedSourceCallbackFacade< Ubitrack::Measurement::ErrorPositionList >)
		//.def("setSourceCallbackErrorPositionList2", &setWrappedSourceCallbackFacade< Ubitrack::Measurement::ErrorPositionList2 >)

		//.def("setSourceCallbackVisionImage", &setWrappedSourceCallbackFacade< Ubitrack::Measurement::ImageMeasurement >)



		.def("addDataflowObserver", &addDataflowObserver)
		.def("removeDataflowObserver", &removeDataflowObserver)
		;

}