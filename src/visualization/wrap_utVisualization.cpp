#include <boost/python.hpp>
#include <boost/python/implicit.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/to_python_converter.hpp>

#include <boost/python/object.hpp>
#include <boost/python/str.hpp>
#include <boost/python/extract.hpp>

#include <utVision/Image.h>

#include "ubitrack_python/BackgroundImage.h"

using namespace Ubitrack;
using namespace Ubitrack::Python;

namespace bp = boost::python;


BOOST_PYTHON_MODULE(_utvisualization)
{
	bp::class_< BackgroundImage, boost::shared_ptr<BackgroundImage>, boost::noncopyable >("BackgroundImage", bp::init<>())
			.def("draw", &BackgroundImage::draw)
			.def("imageIn", &BackgroundImage::imageIn)
			.def("glCleanup", &BackgroundImage::glCleanup)
			;

}
