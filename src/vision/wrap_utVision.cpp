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
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp>


#include <istream>
#include <streambuf>
#include <iostream>

#include <utMeasurement/Measurement.h>
#include <utVision/Image.h>

#include "opencv2/core/core.hpp"

#include <ubitrack_python/wrap_streambuf.h>

using namespace Ubitrack;
using namespace Ubitrack::Python;

namespace bp = boost::python;
#ifndef HAVE_BOOST_NUMPY
namespace bn = boost::python::numpy;
#else
namespace bn = boost::numpy;
#endif

namespace {

// this code will copy the data .. maybe there is a better way to do it
// image-measurement is a parent, whose visibility can be used to manage the array data.

template< typename T >
static bn::ndarray convert_image_as_array(Measurement::ImageMeasurement const & im) {
    bn::dtype typenum = bn::dtype::get_builtin<T>();

	unsigned int frame_bytes = im->width() * im->height() * im->channels();

    bp::tuple shape;
	if (im->channels() == 1) {
    	shape = bp::make_tuple(im->dimension().height, im->dimension().width);
	}
	else if (im->channels() == 3) {
    	shape = bp::make_tuple(im->dimension().height, im->dimension().width, 3);
    }

	bn::ndarray ret = bn::zeros(shape, typenum);
	unsigned char* srcData = (unsigned char*) im->Mat().data;
	unsigned char* dstData = (unsigned char*) ret.get_data();
	memcpy(dstData, srcData, sizeof(unsigned char)*frame_bytes);
	return ret;
}


static bn::ndarray get_image_as_array(bp::object const & self) {
	Measurement::ImageMeasurement im = bp::extract<Measurement::ImageMeasurement const &>(self)();
    switch (im->depth()) {
    case CV_16U:
    	return convert_image_as_array<unsigned short>(im);
    	break;
    case CV_32F:
    	return convert_image_as_array<float>(im);
    	break;
    default:
    	// assume CV8U is the default
    	return convert_image_as_array<unsigned char>(im);
    }
}

} // end anon ns


BOOST_PYTHON_MODULE(_utvision)
{
	// initialize boost.numpy
	bn::initialize();

	//image_to_python_converter().to_python();

	bp::class_<Measurement::ImageMeasurement>("ImageMeasurement")
		.def(bp::init<Measurement::Timestamp>())
		.def("time", (Measurement::Timestamp (Measurement::ImageMeasurement::*)() const)&Measurement::ImageMeasurement::time)
		.def("set_time", (void (Measurement::ImageMeasurement::*)(Measurement::Timestamp))&Measurement::ImageMeasurement::time)
		.def("invalid", (bool (Measurement::ImageMeasurement::*)())&Measurement::ImageMeasurement::invalid)
		.def("invalidate", (void (Measurement::ImageMeasurement::*)())&Measurement::ImageMeasurement::invalidate)
		.def("get", &get_image_as_array)
		// .def("setPixels", some method to transfer the image data to a texture)
		;

}
