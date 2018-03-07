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
#include <utVision/Image.h>

#include "opencv2/core/core.hpp"



void bind_utVisionMain(py::module& m)
{
	typedef Ubitrack::Vision::Image ImageType;

	py::class_<ImageType::Dimension> (m, "Dimension")
		.def_readwrite("width", &ImageType::Dimension::width)
		.def_readwrite("height", &ImageType::Dimension::height)
		.def("__eq__", [](ImageType::Dimension& s, ImageType::Dimension& o){ return s == o; })
		;

	py::class_<ImageType::ImageFormatProperties> (m, "ImageFormatProperties")
		.def_readwrite("imageFormat", &ImageType::ImageFormatProperties::imageFormat)
		.def_readwrite("depth", &ImageType::ImageFormatProperties::depth)
		.def_readwrite("channels", &ImageType::ImageFormatProperties::channels)
		.def_readwrite("matType", &ImageType::ImageFormatProperties::matType)
		.def_readwrite("bitsPerPixel", &ImageType::ImageFormatProperties::bitsPerPixel)
		.def_readwrite("origin", &ImageType::ImageFormatProperties::origin)
		.def_readwrite("align", &ImageType::ImageFormatProperties::align)
		;


	py::class_<ImageType> image(m, "Image", py::buffer_protocol());
	image
		.def(py::init<int, int, ImageType::ImageFormatProperties&, ImageType::ImageUploadState>(),
			py::arg("nWidth"), py::arg("nHeight"), py::arg("nFormat"), py::arg("nState") = ImageType::OnCPU)
        .def_buffer([](ImageType &m) -> py::buffer_info {
        	std::string fmt_descriptor;
        	std::size_t fmt_size;
        	unsigned int fmt_channels;

        	ImageType::ImageFormatProperties fmt;
        	m.getFormatProperties(fmt);

		    // determine datatype
		    switch(fmt.depth) {
		    case CV_8U:
		        fmt_descriptor = py::format_descriptor<unsigned char>::format();
		        fmt_size = sizeof(unsigned char);
		        break;
		    case CV_16U:
		        fmt_descriptor = py::format_descriptor<unsigned short>::format();
		        fmt_size = sizeof(unsigned short);
		        break;
		    case CV_32F:
		        fmt_descriptor = py::format_descriptor<float>::format();
		        fmt_size = sizeof(float);
		        break;
		    case CV_64F:
		        fmt_descriptor = py::format_descriptor<double>::format();
		        fmt_size = sizeof(double);
		        break;
		    default:
		    	throw pybind11::value_error("Unknown image depth");
		    }

			// determine image properties
			switch (fmt.imageFormat) {
			case ImageType::LUMINANCE:
			    fmt_channels = 1;
			    break;
			case ImageType::RGB:
			case ImageType::BGR:
			    fmt_channels = 3;
			    break;
			case ImageType::BGRA:
			case ImageType::RGBA:
			    fmt_channels = 4;
			    break;
			default:
		    	throw pybind11::value_error("Unknown image format");
			}

            if (fmt_channels == 1) {
                    return py::buffer_info(
                    	m.Mat().data, fmt_size, fmt_descriptor, 2,
                    	{static_cast<unsigned long>(m.height()), static_cast<unsigned long>(m.width())},
                    	{static_cast<unsigned long>(fmt_size * fmt_channels * m.width()), static_cast<unsigned long>(fmt_size * fmt_channels)});
            } else {
                    return py::buffer_info(
                    	m.Mat().data, fmt_size, fmt_descriptor, 3,
                    	{static_cast<unsigned long>(m.height()), static_cast<unsigned long>(m.width()), static_cast<unsigned long>(fmt_channels)},
                        {static_cast<unsigned long>(fmt_size * fmt_channels * m.width()),
                            static_cast<unsigned long>(fmt_size * fmt_channels),
                            static_cast<unsigned long>(fmt_size)});
            }
         })
		.def("channels", &ImageType::channels)
		.def("width", &ImageType::width)
		.def("height", &ImageType::height)
		.def("depth", &ImageType::depth)
		.def("bitsPerPixel", &ImageType::bitsPerPixel)
		.def("origin", &ImageType::origin)
		.def("cvMatType", &ImageType::cvMatType)
		.def("pixelFormat", &ImageType::pixelFormat)
		.def("getFormatProperties", &ImageType::getFormatProperties)
		.def("getImageState", &ImageType::getImageState)
		.def("isOnGPU", &ImageType::isOnGPU)
		.def("isOnCPU", &ImageType::isOnCPU)
		.def("saveAsJpeg", &ImageType::saveAsJpeg, py::arg("filename"), py::arg("compressionFactor") = 95)
		;


    py::enum_<ImageType::ImageUploadState>(image, "ImageUploadState")
	    .value("OnCPU", ImageType::OnCPU)
	    .value("OnGPU", ImageType::OnGPU)
	    .value("OnCPUGPU", ImageType::OnCPUGPU)
	    ;

    py::enum_<ImageType::PixelFormat>(image, "PixelFormat")
	    .value("UNKNOWN_PIXELFORMAT", ImageType::UNKNOWN_PIXELFORMAT)
	    .value("LUMINANCE", ImageType::LUMINANCE)
	    .value("RGB", ImageType::RGB)
	    .value("BGR", ImageType::BGR)
	    .value("RGBA", ImageType::RGBA)
	    .value("BGRA", ImageType::BGRA)
	    .value("YUV422", ImageType::YUV422)
	    .value("YUV411", ImageType::YUV411)
	    .value("RAW", ImageType::RAW)
	    .value("DEPTH", ImageType::DEPTH)
	    ;

    py::enum_<ImageType::ImageProperties>(image, "ImageProperties")
	    .value("IMAGE_FORMAT", ImageType::IMAGE_FORMAT)
	    .value("IMAGE_DEPTH", ImageType::IMAGE_DEPTH)
	    .value("IMAGE_CHANNELS", ImageType::IMAGE_CHANNELS)
	    .value("IMAGE_BITSPERPIXEL", ImageType::IMAGE_BITSPERPIXEL)
	    .value("IMAGE_ORIGIN", ImageType::IMAGE_ORIGIN)
	    ;

	py::class_<Ubitrack::Measurement::ImageMeasurement>(m, "ImageMeasurement")
		.def(py::init<Ubitrack::Measurement::Timestamp>())
		// Images are non-copyable
		// .def(py::init<Ubitrack::Measurement::Timestamp, const ImageType& >())
		.def("time", (Ubitrack::Measurement::Timestamp (Ubitrack::Measurement::ImageMeasurement::*)() const)&Ubitrack::Measurement::ImageMeasurement::time)
		.def("set_time", (void (Ubitrack::Measurement::ImageMeasurement::*)(Ubitrack::Measurement::Timestamp))&Ubitrack::Measurement::ImageMeasurement::time)
		.def("invalid", &Ubitrack::Measurement::ImageMeasurement::invalid)
		.def("invalidate", &Ubitrack::Measurement::ImageMeasurement::invalidate)
		.def("get", &Ubitrack::Measurement::ImageMeasurement::get, py::return_value_policy::reference_internal)
		;


}