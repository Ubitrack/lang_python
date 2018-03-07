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

#include <utDataflow/Component.h>
#include <utDataflow/DataflowNetwork.h>
#include <utDataflow/EventQueue.h>
#include <utDataflow/Port.h>
#include <utDataflow/PushConsumer.h>

unsigned int getQueueLength() {
	return Ubitrack::Dataflow::EventQueue::singleton().getCurrentQueueLength();
}

void bind_utDataflowMain(py::module& m)
{

	py::class_< Ubitrack::Dataflow::Component, boost::shared_ptr< Ubitrack::Dataflow::Component >>(m, "Component")
		.def("getName", &Ubitrack::Dataflow::Component::getName, py::return_value_policy::reference_internal)
		.def("addPort", &Ubitrack::Dataflow::Component::addPort)
		.def("removePort", &Ubitrack::Dataflow::Component::removePort)
		.def("getPortByName", &Ubitrack::Dataflow::Component::getPortByName ,py::return_value_policy::reference_internal)
		.def("start", &Ubitrack::Dataflow::Component::start)
		.def("stop", &Ubitrack::Dataflow::Component::stop)
		.def("setEventPriority", &Ubitrack::Dataflow::Component::setEventPriority)
		.def("getEventPriority", &Ubitrack::Dataflow::Component::getEventPriority)
		;

	py::class_< Ubitrack::Dataflow::Port, boost::shared_ptr< Ubitrack::Dataflow::Port >>(m, "Port")
		.def("getName", &Ubitrack::Dataflow::Port::getName ,py::return_value_policy::reference_internal)
		.def("fullName", &Ubitrack::Dataflow::Port::fullName)
		.def("getComponent", &Ubitrack::Dataflow::Port::getComponent ,py::return_value_policy::reference_internal)
		.def("connect", &Ubitrack::Dataflow::Port::connect)
		.def("disconnect", &Ubitrack::Dataflow::Port::disconnect)
		;

	m.def("getEventQueueLength", &getQueueLength);


}