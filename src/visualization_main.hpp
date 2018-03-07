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

#include <utVision/Image.h>

#include "ubitrack_python/BackgroundImage.h"


void bind_utVisualizationMain(py::module& m)
{
	py::class_< Ubitrack::Python::BackgroundImage, boost::shared_ptr<Ubitrack::Python::BackgroundImage> >(m, "BackgroundImage")
		.def(py::init<>())
		.def("draw", &Ubitrack::Python::BackgroundImage::draw)
		.def("imageIn", &Ubitrack::Python::BackgroundImage::imageIn)
		.def("glCleanup", &Ubitrack::Python::BackgroundImage::glCleanup)
		;
}