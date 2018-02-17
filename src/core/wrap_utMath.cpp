#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <complex>
#include <utMath/Vector.h>
#include <utMath/Matrix.h>
#include <utMath/Quaternion.h>
#include <utMath/Pose.h>
#include <utMath/RotationVelocity.h>
#include <utMath/CameraIntrinsics.h>


#include <utMeasurement/Measurement.h>

// custom functions wrapped from utMath
#include <utMath/Stochastic/Average.h>


using namespace Ubitrack;

namespace py = pybind11;

template <typename T>
using Vector2 = Math::Vector< T, 2>;

template <typename T>
using Vector3 = Math::Vector< T, 3>;

template <typename T>
using Vector4 = Math::Vector< T, 4>;

// type caster: Matrix <-> NumPy-array
namespace pybind11 { namespace detail {

  template <typename T> struct type_caster<Vector2<T>>
  {
    public:

      PYBIND11_TYPE_CASTER(Vector2<T>, _("Vector2<T>"));

      // Conversion part 1 (Python -> C++)
      bool load(py::handle src, bool convert)
      {
        if (!convert && !py::array_t<T>::check_(src))
          return false;

        auto buf = py::array_t<T, py::array::c_style | py::array::forcecast>::ensure(src);
        if (!buf)
          return false;

        auto dims = buf.ndim();
        if (dims != 1)
          return false;

        auto size = buf.size();
        if (size != 2)
          return false;

      	Vector2<T> value;

        for ( int i=0 ; i<2 ; i++ )
          value(i) = buf[i];

        return true;
      }

      //Conversion part 2 (C++ -> Python)
      static py::handle cast(const Vector2<T>& src,
        py::return_value_policy policy, py::handle parent)
      {
        py::array_t<T> a({2}, {2*sizeof(T)}, src.content() );

        return a.release();
      }
  };

  template <typename T> struct type_caster<Math::Vector<T, 3> >
  {
    public:

      PYBIND11_TYPE_CASTER(Vector3<T>, _("Vector3<T>"));

      // Conversion part 1 (Python -> C++)
      bool load(py::handle src, bool convert)
      {
        if (!convert && !py::array_t<T>::check_(src))
          return false;

        auto buf = py::array_t<T, py::array::c_style | py::array::forcecast>::ensure(src);
        if (!buf)
          return false;

        auto dims = buf.ndim();
        if (dims != 1)
          return false;

        auto size = buf.size();
        if (size != 3)
          return false;

      	Vector3<T> value;

        for ( int i=0 ; i<3 ; i++ )
          value(i) = buf[i];

        return true;
      }

      //Conversion part 2 (C++ -> Python)
      static py::handle cast(const Vector3<T>& src,
        py::return_value_policy policy, py::handle parent)
      {
        py::array_t<T> a({3}, {3*sizeof(T)}, src.content() );

        return a.release();
      }
  };

  template <typename T> struct type_caster<Vector4<T>>
  {
    public:

      PYBIND11_TYPE_CASTER(Vector4<T>, _("Math::Vector<T, 4>"));

      // Conversion part 1 (Python -> C++)
      bool load(py::handle src, bool convert)
      {
        if (!convert && !py::array_t<T>::check_(src))
          return false;

        auto buf = py::array_t<T, py::array::c_style | py::array::forcecast>::ensure(src);
        if (!buf)
          return false;

        auto dims = buf.ndim();
        if (dims != 1)
          return false;

        auto size = buf.size();
        if (size != 4)
          return false;

      	Vector4<T> value;

        for ( int i=0 ; i<4 ; i++ )
          value(i) = buf[i];

        return true;
      }

      //Conversion part 2 (C++ -> Python)
      static py::handle cast(const Vector4<T>& src,
        py::return_value_policy policy, py::handle parent)
      {
        py::array_t<T> a({4}, {4*sizeof(T)}, src.content() );

        return a.release();
      }
  };

}}




// namespace {

// // by value converters for vector/matrix

// // Converts a vector < n > to numpy array (copy)
// template<typename T, int N>
// struct vector_to_array {
// 	static PyObject* convert(Math::Vector<T, N> const& p) {
// 		bp::tuple shape = bp::make_tuple(N);
// 		bn::ndarray ret = bn::zeros(shape, bn::dtype::get_builtin<T>());
// 		for (int i = 0; i < N; ++i) {
// 			ret[i] = (T) p(i);
// 		}
// 		return bp::incref(ret.ptr());
// 	}

// };

// struct vector_to_python_converter {
// 	template<typename T, int N>
// 	vector_to_python_converter& to_python() {
// 		bp::to_python_converter<Math::Vector<T, N>, vector_to_array<T, N>, false //vector_to_array has no get_pytype
// 		>();
// 		return *this;
// 	}
// };

// // Converts a matrix < n, m > to numpy array (copy)
// template<typename T, int N, int M>
// struct matrix_to_ndarray {
// 	static PyObject* convert(Math::Matrix<T, N, M> const& p) {
// 		bp::tuple shape = bp::make_tuple(N, M);
// 		bn::ndarray ret = bn::zeros(shape, bn::dtype::get_builtin<T>());
// 		Py_intptr_t const * strides = ret.get_strides();
// 		for (int i = 0; i < N; ++i) {
// 			for (int j = 0; j < M; ++j) {
// 				*reinterpret_cast<T *>(ret.get_data() + i * strides[0]
// 						+ j * strides[1]) = p(i, j);
// 			}
// 		}
// 		return bp::incref(ret.ptr());
// 	}

// };

// struct matrix_to_python_converter {
// 	template<typename T, int N, int M>
// 	matrix_to_python_converter& to_python() {
// 		bp::to_python_converter<Math::Matrix<T, N, M>,
// 				matrix_to_ndarray<T, N, M>, false //vector_to_array has no get_pytype
// 		>();
// 		return *this;
// 	}
// };

// // convert ndarray to vector (copy)

// template<typename T, int N>
// static void copy_ndarray_to_vector(bn::ndarray const & array,
// 		Math::Vector<T, N>& vec) {
// 	for (int i = 0; i < N; ++i) {
// 		vec(i) = bp::extract<T>(array[i]);
// 	}
// }

// template<typename T, int N>
// struct vector_from_python {

// 	/**
// 	 *  Register the converter.
// 	 */
// 	vector_from_python() {
// 		bp::converter::registry::push_back(&convertible, &construct,
// 				bp::type_id<Math::Vector<T, N> >());
// 	}

// 	/**
// 	 *  Test to see if we can convert this to the desired type; if not return zero.
// 	 *  If we can convert, returned pointer can be used by construct().
// 	 */
// 	static void * convertible(PyObject * p) {
// 		try {
// 			bp::object obj(bp::handle<>(bp::borrowed(p)));
// 			std::auto_ptr<bn::ndarray> array(
// 					new bn::ndarray(
// 							bn::from_object(obj, bn::dtype::get_builtin<T>(), 1,
// 									1, bn::ndarray::V_CONTIGUOUS)));
// 			if (array->shape(0) != N)
// 				return 0;
// 			return array.release();
// 		} catch (bp::error_already_set & err) {
// 			bp::handle_exception();
// 			return 0;
// 		}
// 	}

// 	/**
// 	 *  Finish the conversion by initializing the C++ object into memory prepared by Boost.Python.
// 	 */
// 	static void construct(PyObject * obj,
// 			bp::converter::rvalue_from_python_stage1_data * data) {
// 		// Extract the array we passed out of the convertible() member function.
// 		std::auto_ptr<bn::ndarray> array(
// 				reinterpret_cast<bn::ndarray*>(data->convertible));
// 		// Find the memory block Boost.Python has prepared for the result.
// 		typedef bp::converter::rvalue_from_python_storage<Math::Vector<T, N> > storage_t;
// 		storage_t * storage = reinterpret_cast<storage_t*>(data);
// 		// Use placement new to initialize the result.
// 		Math::Vector<T, N>* v =
// 				new (storage->storage.bytes) Math::Vector<T, N>();
// 		// Fill the result with the values from the NumPy array.
// 		copy_ndarray_to_vector<T, N>(*array, *v);
// 		// Finish up.
// 		data->convertible = storage->storage.bytes;
// 	}

// };

// // convert ndarray to matrix (copy)

// template<typename T, int N, int M>
// static void copy_ndarray_to_matrix(bn::ndarray const & array,
// 		Math::Matrix<T, N, M>& mat) {
// 	Py_intptr_t const * strides = array.get_strides();
// 	if ((array.shape(0) == N) && (array.shape(1) == M)) {
// 		for (int i = 0; i < N; ++i) {
// 			for (int j = 0; j < M; ++j) {
// 				mat(i, j) = *reinterpret_cast<T const *>(array.get_data()
// 						+ i * strides[0] + j * strides[1]);
// 			}
// 		}
// 	} else {
// 		std::cout << "Matrix size mismatch - should raise an error here .. "
// 				<< std::endl;
// 	}
// }

// template<typename T, int N, int M>
// struct matrix_from_python {

// 	/**
// 	 *  Register the converter.
// 	 */
// 	matrix_from_python() {
// 		bp::converter::registry::push_back(&convertible, &construct,
// 				bp::type_id<Math::Matrix<T, N, M> >());
// 	}

// 	/**
// 	 *  Test to see if we can convert this to the desired type; if not return zero.
// 	 *  If we can convert, returned pointer can be used by construct().
// 	 */
// 	static void * convertible(PyObject * p) {
// 		try {
// 			bp::object obj(bp::handle<>(bp::borrowed(p)));
// 			std::auto_ptr<bn::ndarray> array(
// 					new bn::ndarray(
// 							bn::from_object(obj, bn::dtype::get_builtin<T>(), 2,
// 									2, bn::ndarray::V_CONTIGUOUS)));
// 			if (array->shape(0) != N)
// 				return 0;
// 			if (array->shape(1) != M)
// 				return 0;
// 			return array.release();
// 		} catch (bp::error_already_set & err) {
// 			bp::handle_exception();
// 			return 0;
// 		}
// 	}

// 	/**
// 	 *  Finish the conversion by initializing the C++ object into memory prepared by Boost.Python.
// 	 */
// 	static void construct(PyObject * obj,
// 			bp::converter::rvalue_from_python_stage1_data * data) {
// 		// Extract the array we passed out of the convertible() member function.
// 		std::auto_ptr<bn::ndarray> array(
// 				reinterpret_cast<bn::ndarray*>(data->convertible));
// 		// Find the memory block Boost.Python has prepared for the result.
// 		typedef bp::converter::rvalue_from_python_storage<Math::Matrix<T, N, M> > storage_t;
// 		storage_t * storage = reinterpret_cast<storage_t*>(data);
// 		// Use placement new to initialize the result.
// 		Math::Matrix<T, N, M>* v = new (storage->storage.bytes) Math::Matrix<T, N, M>();
// 		// Fill the result with the values from the NumPy array.
// 		copy_ndarray_to_matrix<T, N, M>(*array, *v);
// 		// Finish up.
// 		data->convertible = storage->storage.bytes;
// 	}

// };


// // exposer for ErrorVectors

// template< class T, int N >
// static bn::ndarray get_ev_value(bp::object const & self) {
// 	Math::Vector< double, N> const & v = bp::extract<T const &>(self)().value;
// 	return bn::from_data(v.content(), bn::dtype::get_builtin<double>(),
// 			bp::make_tuple(N), bp::make_tuple(sizeof(double)), self);
// }

// template< class T, int N >
// static bn::ndarray get_ev_covariance(bp::object const & self) {
// 	Math::Matrix<double, N, N> const & v = bp::extract<T const &>(self)().covariance;
// 	return bn::from_data(v.content(), bn::dtype::get_builtin<double>(),
// 			bp::make_tuple(N, N), bp::make_tuple(sizeof(double) * N, sizeof(double)), self);
// }


// template <class T, int N>
// struct errorvector_exposer
// {
//     template <class C>
//     static void expose(C const& c)
//     {
//         const_cast<C&>(c)
// 				.def(bp::init<const Math::Vector< double, N>&, const Math::Matrix<double, N, N>& >())
// 				.def("getRMS", &T::getRMS)
// 				.def("value", &get_ev_value< T, N >)
// 				.def("covariance", &get_ev_covariance< T, N >)
// 				.def(bp::self_ns::str(bp::self_ns::self))
//             ;
//     }
// };




// // accessor methods for retrieving vectors and matrices

// template<class C, typename T>
// static bn::ndarray py_get_translation(bp::object const & self) {
// 	Math::Vector< T, 3 > const & v = bp::extract<C const &>(self)().translation();
// 	return bn::from_data(v.content(), bn::dtype::get_builtin<T>(),
// 			bp::make_tuple(3), bp::make_tuple(sizeof(T)), self);
// }

// template<class C, typename T>
// static bn::ndarray py_get_covariance(bp::object const & self) {
// 	Math::Matrix< T, 6, 6 > const & v = bp::extract<C const &>(self)().covariance();
// 	return bn::from_data(v.content(), bn::dtype::get_builtin<T>(),
// 			bp::make_tuple(6, 6), bp::make_tuple(sizeof(T) * 6, sizeof(T)), self);
// }


// template<class C, int N, typename T>
// static bn::ndarray py_to_vector(bp::object const & self) {
// 	Math::Vector<T, N> v;
// 	bp::extract<C const &>(self)().toVector(v);
// 	bp::tuple shape = bp::make_tuple(N);
// 	bn::ndarray ret = bn::zeros(shape, bn::dtype::get_builtin<T>());
// 	for (int i = 0; i < N; ++i) {
// 		ret[i] = (T) v(i);
// 	}
// 	return ret;
// }

// template<class C, typename T>
// static bn::ndarray py_to_matrix4x4(bp::object const & self) {
// 	Math::Matrix< T, 4, 4 > m((const Math::Pose&)bp::extract<C const &>(self)());
// 	bp::tuple shape = bp::make_tuple(4,4);
// 	bn::ndarray ret = bn::zeros(shape, bn::dtype::get_builtin<T>());
// 	Py_intptr_t const * strides = ret.get_strides();
// 	for (int i = 0; i < 4; ++i) {
// 		for (int j = 0; j < 4; j++) {
// 			*reinterpret_cast<T *>(ret.get_data() + i * strides[0]
// 									+ j * strides[1]) = m(i, j);
// 		}
// 	}
// 	return ret;
// }

// // helpers
// template<class T>
// T quaternion_from_vector(const Math::Vector< double, 4 >& vec) {
// 	return T::fromVector(vec);
// }

// template<class T>
// T quaternion_from_logarithm(const Math::Vector< double, 3 >& vec) {
// 	return T::fromLogarithm(vec);
// }


// template<class T>
// T quaternion_from_matrix(const Math::Matrix< double, 3, 3 >& m) {
// 	boost::numeric::ublas::matrix< double > um(m);
// 	return T(um);
// }



// template<class T>
// Math::Vector< double, 7 > pose_to_vector(const T& q) {
// 	Math::Vector< double, 7 > vec;
// 	q.toVector(vec);
// 	return vec;
// }

// template<class T>
// T pose_from_vector(const Math::Vector< double, 7 >& vec) {
// 	return T::fromVector(vec);
// }

// template<class T>
// Math::Matrix< double, 3, 3 > quaternion_to_matrix(const T& q) {
// 	Math::Matrix< double, 3, 3 > mat;
// 	q.toMatrix(mat);
// 	return mat;
// }

// template<class T>
// bp::tuple quaternion_to_axisangle(T& q) {
// 	Math::Vector< double, 3 > axis;
// 	double angle;
// 	q.toAxisAngle(axis, angle);
// 	return bp::make_tuple(axis, angle);
// }

// template<typename T>
// T get_value_from_scalar(Math::Scalar<T>& v) {
// 	return v.m_value;
// }

// template<class T>
// Math::Vector< double, 3 > rotationvelocity_to_vector(const T& rv) {
// 	Math::Vector< double, 3 > vec(rv);
// 	return vec;
// }


// template< class EventType, class ResultType >
// ResultType calculate_average(boost::python::list elist) {
// 	 std::vector< EventType > elvec;
// 	 boost::python::ssize_t len = boost::python::len(elist);
// 	 for(int i=0; i<len;i++){
// 		 elvec.push_back(boost::python::extract<EventType>(elist[i]));
// 	 }

// 	 Math::Stochastic::Average< ResultType > avg;
//      avg = std::for_each(elvec.begin(), elvec.end(), avg);
// 	 return avg.getAverage();
// }



// template< typename T >
// static std::vector< T > construct_listtype(boost::python::list data) {
// 	std::vector< T > plist;
// 	boost::python::ssize_t len = boost::python::len(data);
// 	for(int i=0; i<len;i++){
// 		plist.push_back(boost::python::extract< T >(data[i]));
// 	}
// 	return plist;
// }

// Math::Vector< double, 3 > quaternion_transformVector(Math::Quaternion* q, const Math::Vector< double, 3 > &v) {
// 	Math::Vector< double, 3 > r = (*q) * v;
// 	return r;
// }

// }

// // camera intrinsics
// template<class T>
// Math::Matrix< double, 3, 3 > intrinsics_get_matrix(const Math::CameraIntrinsics<T>& i) {
// 	Math::Matrix< double, 3, 3 > mat(i.matrix);
// 	return mat;
// }

// template<class T>
// Math::Vector< double, 2 > intrinsics_get_dimension(const Math::CameraIntrinsics<T>& i) {
// 	Math::Vector< double, 2 > vec(i.dimension);
// 	return vec;
// }

// template<class T>
// Math::Vector< double, 6 > intrinsics_get_radial_params(const Math::CameraIntrinsics<T>& i) {
// 	Math::Vector< T, 6 > vec(i.radial_params);
// 	return vec;
// }

// template<class T>
// Math::Vector< double, 2 > intrinsics_get_tangential_params(const Math::CameraIntrinsics<T>& i) {
// 	Math::Vector< T, 2 > vec(i.tangential_params);
// 	return vec;
// }


// tests
Math::Vector< double, 4 > test_vec4() {
	return Math::Vector< double, 4 >(1, 2, 2, 1.2);
}

// Math::Matrix< double, 3, 3 > test_mat33() {
// 	Math::Quaternion q;
// 	Math::Matrix< double, 3, 3 > m;
// 	q.toMatrix(m);
// 	return m;
// }

// Math::Quaternion test_quat() {
// 	Math::Quaternion q;
// 	return q;
// }

PYBIND11_MODULE(_utmath,m)
{


// 	/*
// 	 * Scalar Classes
// 	 */

// 	bp::class_<Math::Scalar<unsigned long>,
// 			boost::shared_ptr<Math::Scalar<unsigned long> > >("ScalarID",
// 			bp::init<unsigned long>()).add_property("value",
// 			&get_value_from_scalar<unsigned long>).def(
// 			bp::self_ns::str(bp::self_ns::self));

// 	bp::class_<Math::Scalar<int>, boost::shared_ptr<Math::Scalar<int> > >(
// 			"ScalarInt", bp::init<int>()).add_property("value",
// 			&get_value_from_scalar<int>).def(
// 			bp::self_ns::str(bp::self_ns::self));

// 	bp::class_<Math::Scalar<double>, boost::shared_ptr<Math::Scalar<double> > >(
// 			"ScalarDouble", bp::init<double>()).add_property("value",
// 			&get_value_from_scalar<double>).def(
// 			bp::self_ns::str(bp::self_ns::self));

// 	/*
// 	 * Vector Classes
// 	 */

// 	vector_from_python<double, 2>();
// 	vector_from_python<double, 3>();
// 	vector_from_python<double, 4>();
// 	vector_from_python<double, 5>();
// 	vector_from_python<double, 6>();
// 	vector_from_python<double, 7>();
// 	vector_from_python<double, 8>();

// 	vector_to_python_converter()
// 		.to_python<double, 2>()
// 		.to_python<double, 3>()
// 		.to_python<double, 4>()
// 		.to_python<double, 5>()
// 		.to_python<double, 6>()
// 		.to_python<double, 7>()
// 		.to_python<double, 8>();

// 	/*
// 	 * Matrix Classes
// 	 */

// 	matrix_from_python<double, 3, 3>();
// 	matrix_from_python<double, 3, 4>();
// 	matrix_from_python<double, 4, 4>();
// 	matrix_from_python<double, 5, 5>();
// 	matrix_from_python<double, 6, 6>();
// 	matrix_from_python<double, 7, 7>();

// 	matrix_to_python_converter()
// 		.to_python<double, 3, 3>()
// 		.to_python<double, 3, 4>()
// 		.to_python<double, 4, 4>()
// 		.to_python<double, 5, 5>()
// 		.to_python<double, 6, 6>()
// 		.to_python<double, 7, 7>();


// 	errorvector_exposer<Math::ErrorVector< double, 2 >, 2 >::expose(
// 			bp::class_< Math::ErrorVector< double, 2 >, boost::shared_ptr< Math::ErrorVector< double, 2 > > >("ErrorVector2")
// 			);

// 	errorvector_exposer<Math::ErrorVector< double, 3 >, 3 >::expose(
// 			bp::class_< Math::ErrorVector< double, 3 >, boost::shared_ptr< Math::ErrorVector< double, 3 > > >("ErrorVector3")
// 			);

// 	errorvector_exposer<Math::ErrorVector< double, 7 >, 7 >::expose(
// 			bp::class_< Math::ErrorVector< double, 7 >, boost::shared_ptr< Math::ErrorVector< double, 7 > > >("ErrorVector7")
// 			);



// 	/*
// 	 * Quaternion Class
// 	 */

// 	bp::class_<boost::math::quaternion<double>, boost::shared_ptr<boost::math::quaternion<double> > >("QuaternionBase")
// 			.def(bp::init<std::complex<double>&, std::complex<double>&>())
// 			.def(bp::init<double, double, double, double>())

// 			// does not work .. needs casting ??
// 			.def(bp::self += double()).def(bp::self += std::complex<double>()).def(
// 					bp::self += bp::self)

// 			.def(bp::self -= double()).def(bp::self -= std::complex<double>()).def(
// 					bp::self -= bp::self)

// 			.def(bp::self *= double()).def(bp::self *= std::complex<double>()).def(
// 					bp::self *= bp::self)

// 			.def(bp::self /= double()).def(bp::self /= std::complex<double>()).def(
// 					bp::self /= bp::self)

// 			.def(bp::self + bp::self).def(bp::self - bp::self)

// 			.def(bp::self == double()).def(bp::self == std::complex<double>()).def(
// 					bp::self == bp::self)

// 			.def(bp::self != double()).def(bp::self != std::complex<double>()).def(
// 					bp::self != bp::self)

// 			.def("real",
// 					(double (boost::math::quaternion<double>::*)())&boost::math::quaternion< double >::real).def("unreal",(boost::math::quaternion< double > (boost::math::quaternion< double >::*)())&boost::math::quaternion< double >::unreal)

// 			.def(bp::self_ns::str(bp::self_ns::self))
// 			// example for static method ..
// 			//.def("zeros", (Vector2 (*)())&Vector2::zeros)
// 			;

// 	bp::def("sup", &boost::math::sup<double>);
// 	bp::def("l1", &boost::math::l1<double>);
// 	bp::def("abs", &boost::math::abs<double>);
// 	bp::def("conj", &boost::math::conj<double>);
// 	bp::def("norm", &boost::math::norm<double>);
// 	bp::def("spherical", &boost::math::spherical<double>);
// 	bp::def("semipolar", &boost::math::semipolar<double>);
// 	bp::def("multipolar", &boost::math::multipolar<double>);
// 	bp::def("cylindrospherical", &boost::math::cylindrospherical<double>);
// 	bp::def("cylindrical", &boost::math::cylindrical<double>);
// 	bp::def("exp", &boost::math::exp<double>);
// 	bp::def("cos", &boost::math::cos<double>);
// 	bp::def("sin", &boost::math::sin<double>);
// 	bp::def("tan", &boost::math::tan<double>);
// 	bp::def("cosh", &boost::math::cosh<double>);
// 	bp::def("sinh", &boost::math::sinh<double>);
// 	bp::def("tanh", &boost::math::tanh<double>);
// 	bp::def("pow", &boost::math::pow<double>);

// 	{
// 		bp::scope in_Quaternion =
// 				bp::class_<Math::Quaternion, boost::shared_ptr<Math::Quaternion>, bp::bases<boost::math::quaternion<double> > >("Quaternion")
// 					.def(bp::init<const Math::Vector< double, 3 >&, const double>())
// 					.def(bp::init<const boost::math::quaternion<double>& >())
// 					.def(bp::init<double, double, double>())
// 					.def(bp::init<double, double, double, double>())
// 					.def("x",&Math::Quaternion::x)
// 					.def("y", &Math::Quaternion::y)
// 					.def("z", &Math::Quaternion::z)
// 					.def("w",&Math::Quaternion::w)
// 					.def("normalize",(Math::Quaternion& (Math::Quaternion::*)())&Math::Quaternion::normalize,
// 							bp::return_internal_reference<>())
// 					.def("invert", (Math::Quaternion& (Math::Quaternion::*)())&Math::Quaternion::invert,
// 							bp::return_internal_reference<>())
// 					.def("inverted", (Math::Quaternion (Math::Quaternion::*)())&Math::Quaternion::operator~)

// 					// doew not work .. needs casting ??
// 					.def(bp::self += double())
// 					.def(bp::self += std::complex<double>())
// 					.def(bp::self += bp::self)
// 					.def(bp::self += boost::math::quaternion< double >())

// 					.def(bp::self -= double())
// 					.def(bp::self -= std::complex<double>())
// 					.def(bp::self -= bp::self)
// 					.def(bp::self -= boost::math::quaternion< double >())

// 					.def(bp::self *= double())
// 					.def(bp::self *= std::complex<double>())
// 					.def(bp::self *= bp::self)
// 					.def(bp::self *= boost::math::quaternion< double >())

// 					.def(bp::self /= double())
// 					.def(bp::self /= std::complex<double>())
// 					.def(bp::self /= bp::self)
// 					.def(bp::self /= boost::math::quaternion< double >())

// 					.def(bp::self + bp::self)
// 					.def(bp::self - bp::self)
// 					.def(bp::self * bp::self)
// 					.def(bp::self / bp::self)

// 					.def(boost::math::quaternion< double >() + bp::self)
// 					.def(boost::math::quaternion< double >() - bp::self)
// 					.def(boost::math::quaternion< double >() * bp::self)
// 					.def(boost::math::quaternion< double >() / bp::self)

// 					.def(bp::self + boost::math::quaternion< double >())
// 					.def(bp::self - boost::math::quaternion< double >())
// 					.def(bp::self * boost::math::quaternion< double >())
// 					.def(bp::self / boost::math::quaternion< double >())

// 					//.def("transformVector", (Math::Vector< double, 3 > (Math::Quaternion::*)(const Math::Vector< double, 3 > &))&Math::Quaternion::operator*)
// 					.def("transformVector", &quaternion_transformVector)

// 					.def(bp::self == double())
// 					.def(bp::self == std::complex<double>())
// 					.def(bp::self == bp::self)
// 					.def(bp::self == boost::math::quaternion< double >())

// 					.def(bp::self != double())
// 					.def(bp::self != std::complex<double>())
// 					.def(bp::self != bp::self)
// 					.def(bp::self != boost::math::quaternion< double >())

// 					.def("angle", &Math::Quaternion::angle)

// 					.def("negateIfCloser", &Math::Quaternion::negateIfCloser)

// 					.def("toLogarithm", (Math::Vector< double, 3 > (Math::Quaternion::*)())&Math::Quaternion::toLogarithm)
// 					.def("fromLogarithm", &quaternion_from_logarithm<Math::Quaternion>)
// 					.staticmethod("fromLogarithm")

// 					.def("getEulerAngles", (Math::Vector< double, 3 > (Math::Quaternion::*)(Math::Quaternion::t_EulerSequence) const)&Math::Quaternion::getEulerAngles)

// 					.def("toMatrix", &quaternion_to_matrix<Math::Quaternion>)
// 					.def("fromMatrix", &quaternion_from_matrix<Math::Quaternion>)
// 					.staticmethod("fromMatrix")

// 					.def("toAxisAngle", &quaternion_to_axisangle<Math::Quaternion>)

// 					.def("toVector", &py_to_vector<Math::Quaternion, 4, double>)
// 					.def("fromVector", &quaternion_from_vector<Math::Quaternion>)
// 					.staticmethod("fromVector")
// 					;

// 					bp::enum_<Math::Quaternion::t_EulerSequence>("EULER_SEQUENCE")
// 					.value("XYZ", Math::Quaternion::EULER_SEQUENCE_XYZ)
// 					.value("YZX", Math::Quaternion::EULER_SEQUENCE_YZX)
// 					.value("ZXY", Math::Quaternion::EULER_SEQUENCE_ZXY)
// 					.value("ZYX", Math::Quaternion::EULER_SEQUENCE_ZYX)
// 					.value("XZY", Math::Quaternion::EULER_SEQUENCE_XZY)
// 					.value("YXZ", Math::Quaternion::EULER_SEQUENCE_YXZ)
// 					;

// 				}

// 	bp::def("slerp", &Math::slerp);

// 	/*
// 	 * Linear Interpolation of basic types
// 	 */
// 	bp::def("linearInterpolatePose", (Math::Pose (*)(const Math::Pose&, const Math::Pose&, double)) &Math::linearInterpolate);
// 	bp::def("linearInterpolateErrorPose", (Math::ErrorPose (*)(const Math::ErrorPose&, const Math::ErrorPose&, double)) &Math::linearInterpolate);
// 	bp::def("linearInterpolateQuaternion", (Math::Quaternion (*)(const Math::Quaternion&, const Math::Quaternion&, double)) &Math::linearInterpolate);
// 	bp::def("linearInterpolateVector2", (Math::Vector< double, 2 > (*)(const Math::Vector< double, 2 >&, const Math::Vector< double, 2 >&, double)) &Math::linearInterpolate);
// 	bp::def("linearInterpolateVector3", (Math::Vector< double, 3 > (*)(const Math::Vector< double, 3 >&, const Math::Vector< double, 3 >&, double)) &Math::linearInterpolate);
// 	bp::def("linearInterpolateVector4", (Math::Vector< double, 4 > (*)(const Math::Vector< double, 4 >&, const Math::Vector< double, 4 >&, double)) &Math::linearInterpolate);
// 	bp::def("linearInterpolateVector5", (Math::Vector< double, 5 > (*)(const Math::Vector< double, 5 >&, const Math::Vector< double, 5 >&, double)) &Math::linearInterpolate);
// 	bp::def("linearInterpolateVector6", (Math::Vector< double, 6 > (*)(const Math::Vector< double, 6 >&, const Math::Vector< double, 6 >&, double)) &Math::linearInterpolate);
// 	bp::def("linearInterpolateVector7", (Math::Vector< double, 7 > (*)(const Math::Vector< double, 7 >&, const Math::Vector< double, 7 >&, double)) &Math::linearInterpolate);
// 	bp::def("linearInterpolateVector8", (Math::Vector< double, 8 > (*)(const Math::Vector< double, 8 >&, const Math::Vector< double, 8 >&, double)) &Math::linearInterpolate);



// 	/*
// 	 * Pose Class
// 	 */

// 	bp::class_<Math::Pose, boost::shared_ptr<Math::Pose> >("Pose", bp::init<const Math::Quaternion&, const Math::Vector< double, 3 >&>())
// 			.def(bp::init<const Math::Matrix< double, 4, 4 >&>())
// 			.def("rotation",(const Math::Quaternion& (Math::Pose::*)())&Math::Pose::rotation
// 					, bp::return_internal_reference<>()
// 			)
// 			.def("translation", &py_get_translation<Math::Pose, double>
// 					//(const Math::Vector< double, 3 >& (Math::Pose::*)())&Math::Pose::translation
// 					//,return_value_policy<copy_const_reference>()
// 					//,return_internal_reference<>()
// 			)

// 			.def("scalePose", &Math::Pose::scalePose)

// 			.def("toVector", &py_to_vector<Math::Pose, 7, double>)
// 			.def("fromVector", &pose_from_vector<Math::Pose>)
// 			.staticmethod("fromVector")

// 			.def("toMatrix", &py_to_matrix4x4<Math::Pose, double>)

// 			.def(bp::self * Math::Vector< double, 3 >())
// 			.def(bp::self * bp::self)
// 			.def(bp::self == bp::self)

// 			.def("invert", (Math::Pose (Math::Pose::*)())&Math::Pose::operator~)
// 			.def(bp::self_ns::str(bp::self_ns::self))
// 			;

// 	/*
// 	 * ErrorPose Class
// 	 */

// 	bp::class_<Math::ErrorPose, boost::shared_ptr<Math::ErrorPose>, bp::bases< Math::Pose > >("ErrorPose", bp::init<const Math::Quaternion&, const Math::Vector< double, 3 >&, const Math::Matrix< double, 6, 6 >& >())
// 			.def("covariance", &py_get_covariance<Math::ErrorPose, double>)

// 			// missing from/toAdditiveErrorVector

// 			.def("invert", (Math::ErrorPose (Math::ErrorPose::*)())&Math::ErrorPose::operator~)
// 			.def(bp::self_ns::str(bp::self_ns::self))
// 			;

// 	/*
// 	 * RotationVelocity Class
// 	 */

// 	bp::class_<Math::RotationVelocity, boost::shared_ptr<Math::RotationVelocity> >("RotationVelocity", bp::init<const Math::Quaternion&, const Math::Quaternion&, double >())
// 			.def(bp::init<double, double, double>())
// 			.def("integrate", (Math::Quaternion (Math::RotationVelocity::*)(double))&Math::RotationVelocity::integrate)
// 			.def("angularVelocity", (double (Math::RotationVelocity::*)())&Math::RotationVelocity::angularVelocity)
// 			.def("axis", (Math::Vector< double, 3 > (Math::RotationVelocity::*)())&Math::RotationVelocity::axis)
// 			.def("toVector", &rotationvelocity_to_vector<Math::RotationVelocity>)
// 			.def(bp::self_ns::str(bp::self_ns::self))
// 			;




// 	/*
// 	 * CameraIntrinsics Class
// 	 */

// 	bp::class_<Math::CameraIntrinsics<double>, boost::shared_ptr<Math::CameraIntrinsics<double> > >("CameraIntrinsics", bp::init<const Math::Matrix< double, 3, 3>&, const Math::Vector< double, 2 >&, const Math::Vector< double, 2 >& >())
// 		.def(bp::init<const Math::CameraIntrinsics<double> &>())
// 		.def("intrinsics", &intrinsics_get_matrix<double>)
// 		.def("dimension", &intrinsics_get_dimension<double>)
// 		.def("tangential_params", &intrinsics_get_tangential_params<double>)
// 		.def("radial_params", &intrinsics_get_radial_params<double>)
// 		.def("angleVertical", &Math::CameraIntrinsics<double>::angleVertical)
// 		.def("angleHorizontal", &Math::CameraIntrinsics<double>::angleHorizontal)
// 		.def("angleDiagonal", &Math::CameraIntrinsics<double>::angleDiagonal)
// 		.def(bp::self_ns::str(bp::self_ns::self))
// 		;


// 					// std::vector<T> support
// 	bp::class_<std::vector<Math::Pose> >("PoseList")
// //			.def("__iter__", bp::iterator<std::vector< Math::Vector< double, 3 > > >())
// //			.def("__len__", &std::vector< Math::Vector< double, 3 > >::size)
// 			.def(bp::vector_indexing_suite<std::vector<Math::Pose> >())
// 			.def("fromList", &construct_listtype< Math::Pose >)
// 			.staticmethod("fromList")
// 			;

// 	bp::class_<std::vector<Math::Vector< double, 2 > > >("PositionList2")
// 			.def(bp::vector_indexing_suite<std::vector<Math::Vector< double, 2 > > >())
// 			.def("fromList", &construct_listtype< Math::Vector< double, 2 > >)
// 			.staticmethod("fromList")
// 			;

// 	bp::class_<std::vector<Math::Vector< double, 3 > > >("PositionList")
// //			.def("__iter__",bp::iterator<std::vector<Math::Vector< double, 3 > > >())
// //			.def("__len__",&std::vector<Math::Vector< double, 3 > >::size)
// 			.def(bp::vector_indexing_suite<std::vector< Math::Vector< double, 3 > > >())
// 			.def("fromList", &construct_listtype< Math::Vector< double, 3 > >)
// 			.staticmethod("fromList")
// 			;

// 	bp::class_<std::vector<Math::Scalar<double> > >("DistanceList")
// 			.def(bp::vector_indexing_suite<std::vector<Math::Scalar<double> > >())
// 			;

// 	bp::class_<std::vector<Math::Scalar<unsigned long> > >("IDList")
// 			.def(bp::vector_indexing_suite<std::vector<Math::Scalar<unsigned long> > >())
// 			;

// /*
// 	// currently unavailable due to missing == operator for ErrorVector/Pose
//     bp::class_<std::vector< Math::ErrorPose > >("ErrorPoseList")
// 			.def(bp::vector_indexing_suite<std::vector< Math::ErrorPose > >())
// 			;
//     bp::class_<std::vector< Math::ErrorVector< double, 2 > > >("ErrorPositionList2")
//     		.def(bp::vector_indexing_suite<std::vector< Math::ErrorVector< double, 2 > > >())
//     		;

//     bp::class_<std::vector< Math::ErrorVector< double, 3 > > >("ErrorPositionList")
//     		.def(bp::vector_indexing_suite<std::vector< Math::ErrorVector< double, 3 > > >())
//     		;
// */

// 	// Pose and Position Average with Covariance
// 	bp::def("averagePoseListError", &calculate_average< Math::Pose, Math::ErrorPose>);
// 	bp::def("averagePositionListError", &calculate_average< Math::Vector< double, 3 >, Math::ErrorVector< double, 3 > >);

// 	/*
// 	 * Testing functions
// 	 */

	m.def("test_vec4", test_vec4);
	// bp::def("test_mat33", test_mat33);
	// bp::def("test_quat", test_quat);
}

