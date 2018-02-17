#include <boost/python.hpp>
#include <boost/python/implicit.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/to_python_converter.hpp>

#include <boost/python/object.hpp>
#include <boost/python/str.hpp>
#include <boost/python/extract.hpp>

#include <utVisualization/RenderAPI/utRenderAPI.h>

#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <vector>


using namespace Ubitrack;
using namespace Ubitrack::Visualization;

namespace bp = boost::python;


namespace {

    struct notification_callback_wrapper_t {
        notification_callback_wrapper_t(bp::object callable) : _callable(callable) { }

        bool operator()() {
            PyGILState_STATE gstate = PyGILState_Ensure();
            bool ret = _callable();
            PyGILState_Release(gstate);
            return ret;
        }

        bp::object _callable;
    };


    class PyWindowImpl;

    // baseclass for python-based window implementation
    class WindowBase
    {
    public:

        virtual ~WindowBase() {
//            m_pWindowHandle.reset();
        }

        virtual std::string title()  {
            return "WindowBase";
        }

        virtual int width()  {
            return 0;
        }

        virtual int height()  {
            return 0;
        }

        virtual bool create()  {
            return false;
        }

        virtual void destroy() {
            m_pCameraHandle.reset();
        }


        virtual bool is_valid()  {
            return false;
        }

        virtual bool pre_init_gl()  {
            return true;
        }

        virtual bool post_init_gl()  {
            return false;
        }

        virtual bool pre_render()  {
            return true;
        }

//        virtual bool render_camera(int elapsed_time) {
//            if (m_pCameraHandle) {
//                m_pCameraHandle->render(elapsed_time);
//                return true;
//            }
//            return false;
//        }
//
        virtual bool post_render()  {
            return false;
        }



        // helper methods to integrate with window toolkit
        virtual void on_window_size(int w, int h) {
            if (m_pCameraHandle) {
                m_pCameraHandle->on_window_size(w, h);
            }
        }

        virtual void on_render(int elapsed_time) {
            if (m_pCameraHandle) {
                m_pCameraHandle->on_render(elapsed_time);
            }
        }

        virtual void on_window_close() {
            if (m_pCameraHandle) {
                m_pCameraHandle->on_window_close();
            }
        }

        virtual void on_keypress(int key, int scancode, int action, int mods) {
            if (m_pCameraHandle) {
                m_pCameraHandle->on_keypress(key, scancode, action, mods);
            }
        }

        virtual void on_cursorpos(double xpos, double ypos) {
            if (m_pCameraHandle) {
                m_pCameraHandle->on_cursorpos(xpos, ypos);
            }
        }

        // integration
        void setCameraHandle(boost::shared_ptr <CameraHandle> &camera_handle) {
            m_pCameraHandle = camera_handle;
        }

//        void setWindowHandle(boost::shared_ptr <PyWindowImpl> &window_handle) {
//            m_pWindowHandle = window_handle;
//        }


    private:
//        boost::shared_ptr <PyWindowImpl> m_pWindowHandle;
        boost::shared_ptr <CameraHandle> m_pCameraHandle;
    };

    struct WindowBaseWrap : WindowBase, bp::wrapper<WindowBase>
    {
        // wrap title
        virtual std::string title()
        {
            if (bp::override n = this->get_override("title"))
                return n();
            return WindowBase::title();
        }
        std::string default_title()
        {
            return this->WindowBase::title();
        }


        // wrap width
        virtual int width()
        {
            if (bp::override n = this->get_override("width"))
                return n();
            return WindowBase::width();
        }
        int default_width()
        {
            return this->WindowBase::width();
        }

        // wrap height
        virtual int height()
        {
            if (bp::override n = this->get_override("height"))
                return n();
            return WindowBase::height();
        }
        int default_height()
        {
            return this->WindowBase::height();
        }

        // wrap create
        virtual bool create()
        {
            if (bp::override n = this->get_override("create"))
                return n();
            return WindowBase::create();
        }
        bool default_create()
        {
            return this->WindowBase::create();
        }

        // wrap destroy
        virtual void destroy()
        {
            if (bp::override n = this->get_override("destroy")) {
                n();
                return;
            }
            WindowBase::destroy();
        }
        void default_destroy()
        {
            this->WindowBase::destroy();
        }

        // wrap is_valid
        virtual bool is_valid()
        {
            if (bp::override n = this->get_override("is_valid"))
                return n();
            return WindowBase::is_valid();
        }
        bool default_is_valid()
        {
            return this->WindowBase::is_valid();
        }

        // wrap pre_init_gl
        virtual bool pre_init_gl()
        {
            if (bp::override n = this->get_override("pre_init_gl"))
                return n();
            return WindowBase::pre_init_gl();
        }
        bool default_pre_init_gl()
        {
            return this->WindowBase::pre_init_gl();
        }

        // wrap post_init_gl
        virtual bool post_init_gl()
        {
            if (bp::override n = this->get_override("post_init_gl"))
                return n();
            return WindowBase::post_init_gl();
        }
        bool default_post_init_gl()
        {
            return this->WindowBase::post_init_gl();
        }

        // wrap pre_render
        virtual bool pre_render()
        {
            if (bp::override n = this->get_override("pre_render"))
                return n();
            return WindowBase::pre_render();
        }
        bool default_pre_render()
        {
            return this->WindowBase::pre_render();
        }

        // wrap post_render
        virtual bool post_render()
        {
            if (bp::override n = this->get_override("post_render"))
                return n();
            return WindowBase::post_render();
        }
        bool default_post_render()
        {
            return this->WindowBase::post_render();
        }


        // wrap on_window_size
        virtual void on_window_size(int w, int h)
        {
            if (bp::override n = this->get_override("on_window_size")) {
                n(w, h);
                return;
            }
            WindowBase::on_window_size(w, h);
        }
        void default_on_window_size(int w, int h)
        {
            this->WindowBase::on_window_size(w, h);
        }

        // wrap on_render
        virtual void on_render(int elapsed_time)
        {
            if (bp::override n = this->get_override("on_render")) {
                n(elapsed_time);
                return;
            }
            WindowBase::on_render(elapsed_time);
        }
        void default_on_render(int elapsed_time)
        {
            this->WindowBase::on_render(elapsed_time);
        }

        // wrap on_window_close
        virtual void on_window_close()
        {
            if (bp::override n = this->get_override("on_window_close")) {
                n();
                return;
            }
            WindowBase::on_window_close();
        }
        void default_on_window_close()
        {
            this->WindowBase::on_window_close();
        }

        // wrap on_keypress
        virtual void on_keypress(int key, int scancode, int action, int mods)
        {
            if (bp::override n = this->get_override("on_keypress")) {
                n(key, scancode, action, mods);
                return;
            }
            WindowBase::on_keypress(key, scancode, action, mods);
        }
        void default_on_keypress(int key, int scancode, int action, int mods)
        {
            this->WindowBase::on_keypress(key, scancode, action, mods);
        }

        // wrap on_cursorpos
        virtual void on_cursorpos(double xpos, double ypos)
        {
            if (bp::override n = this->get_override("on_cursorpos")) {
                n(xpos, ypos);
                return;
            }
            WindowBase::on_cursorpos(xpos, ypos);
        }
        void default_on_cursorpos(double xpos, double ypos)
        {
            this->WindowBase::on_cursorpos(xpos, ypos);
        }

    };


// Python Window implementation
    class PyWindowImpl : public VirtualWindow {

    public:
        PyWindowImpl(int _width, int _height, const std::string &_title, WindowBase* _handle)
        : VirtualWindow(_width, _height, _title)
        , m_pWindowHandle(_handle)
        {
        }

        ~PyWindowImpl() {
            m_pWindowHandle = NULL;
        }

        virtual bool is_valid() {
            if (m_pWindowHandle) {
                return m_pWindowHandle->is_valid();
            }
            return false;
        }

        virtual bool create() {
            if (m_pWindowHandle) {
                return m_pWindowHandle->create();
            }
            return false;
        }

        virtual void initGL(boost::shared_ptr <CameraHandle> &event_handler) {
            if (m_pWindowHandle) {
                if(m_pWindowHandle->pre_init_gl()) {
                    m_pWindowHandle->setCameraHandle(event_handler);
                    m_pWindowHandle->post_init_gl();
                }
            }
        }

        virtual void destroy() {
            if (m_pWindowHandle) {
                return m_pWindowHandle->destroy();
            }
        }


        bool pre_render() {
            if (m_pWindowHandle) {
                return m_pWindowHandle->pre_render();
            }
            return false;
        }

        bool post_render() {
            if (m_pWindowHandle) {
                return m_pWindowHandle->post_render();
            }
            return false;
        }



    private:
        WindowBase* m_pWindowHandle;
    };
//
//    struct PyWindowWrap : PyWindowImpl, bp::wrapper<PyWindowImpl> {
//        PyWindowWrap(int _width, int _height, const std::string &_title)
//                : PyWindowImpl(_width, _height, _title) {
//
//
//        virtual void pre_render() {
//            std::cout << "pre render" << std::endl;
//            if (bp::override n = this->get_override("pre_render")) {
//                n();
//                return;
//            }
//            PyWindowImpl::pre_render();
//        }
//
//        void default_pre_render() {
//            this->PyWindowImpl::pre_render();
//        }
//
//        virtual void post_render() {
//            std::cout << "post render" << std::endl;
//            if (bp::override n = this->get_override("post_render")) {
//                n();
//            }
//            PyWindowImpl::post_render();
//        }
//
//        void default_post_render() {
//            this->PyWindowImpl::post_render();
//        }
//
//        // Implementation of Public interface
//        virtual bool is_valid() {
//            if (bp::override n = this->get_override("is_valid"))
//                return n();
//            return PyWindowImpl::is_valid();
//        }
//
//        bool default_is_valid() {
//            return this->PyWindowImpl::is_valid();
//        }
//
//        virtual bool create() {
//            std::cout << "create " << std::endl;
//            if (bp::override n = this->get_override("create"))
//                return n();
//            return this->PyWindowImpl::create();
//        }
//
//        void default_create() {
//            this->PyWindowImpl::create();
//        }
//
//        virtual void initGL_callback() {
//            if (bp::override n = this->get_override("initGL_callback")) {
//                n();
//            }
//        }
//
//        virtual void destroy() {
//            std::cout << "destroy" << std::endl;
//            if (bp::override n = this->get_override("destroy")) {
//                n();
//                return;
//            }
//            this->PyWindowImpl::destroy();
//        }
//
//        void default_destroy() {
//            this->PyWindowImpl::destroy();
//        }
//    };
//

    void RM_setup() {
        Visualization::RenderManager &rm = Visualization::RenderManager::singleton();
        rm.setup();
    }

    void RM_teardown() {
        Visualization::RenderManager &rm = Visualization::RenderManager::singleton();
        rm.teardown();
    }

    bool RM_wait_for_event(int timeout) {
        Visualization::RenderManager &rm = Visualization::RenderManager::singleton();
        return rm.wait_for_event(timeout);
    }

    void RM_register_notify_callback(bp::object function) {
        Visualization::RenderManager &rm = Visualization::RenderManager::singleton();
        rm.register_notify_callback(boost::function<void()>(
                notification_callback_wrapper_t(function)));
    }

    void RM_unregister_notify_callback() {
        Visualization::RenderManager &rm = Visualization::RenderManager::singleton();
        rm.unregister_notify_callback();
    }

    bp::object RM_check_cameras_and_setup(bp::object create_window_callback) {
        Visualization::RenderManager &rm = Visualization::RenderManager::singleton();
        bp::object pywin;
        if (rm.need_setup()) {
            bool window_opened = false;
            boost::shared_ptr <CameraHandle> cam = rm.setup_pop_front();
            boost::shared_ptr <PyWindowImpl> win;

            std::cout << "Camera setup: " << cam->title() << std::endl;

            PyGILState_STATE gstate = PyGILState_Ensure();
            pywin = create_window_callback(cam->initial_width(),
                                           cam->initial_height(),
                                           cam->title());
            PyGILState_Release(gstate);

            WindowBase* pywin_ = bp::extract<WindowBase*>(pywin);
            win.reset(new PyWindowImpl(pywin_->width(), pywin_->height(), pywin_->title(), pywin_));

            boost::shared_ptr <VirtualWindow> win_ = boost::dynamic_pointer_cast<VirtualWindow>(win);
            if (!cam->setup(win_)) {
                rm.setup_push_back(cam);
            } else {
                win->initGL(cam);
                window_opened = true;
            }
        }
        return pywin;
    }


    int RM_render_and_cleanup(int elapsed_time) {
        int windows_closed = 0;
        Visualization::RenderManager &rm = Visualization::RenderManager::singleton();
        boost::shared_ptr <CameraHandle> cam;
        boost::shared_ptr <PyWindowImpl> win;

        std::vector<unsigned int> chToDelete;
        CameraHandleMap::iterator pos = rm.cameras_begin();
        CameraHandleMap::iterator end = rm.cameras_end();
        while (pos != end) {
            bool is_valid = false;
            if (pos->second) {
                cam = pos->second;
                if (cam->get_window()) {
                    win = boost::dynamic_pointer_cast<PyWindowImpl>(cam->get_window());
                    if ((win) && (win->is_valid())) {
                        win->pre_render();
//					cam->pre_render();
                        cam->render(elapsed_time);
//                  cam->post_render(); ??
                        is_valid = true;
                        win->post_render();  // make this loop through all current windows??
//                    CheckForGLErrors("Render Error");
                    }
                }
            }
            pos++;
            if (!is_valid) {
                chToDelete.push_back(pos->first);
            }
        }

        if (chToDelete.size() > 0) {
            for (unsigned int i = 0; i < chToDelete.size(); i++) {
                unsigned int cam_id = chToDelete.at(i);
                rm.get_camera(cam_id)->teardown();
                rm.unregister_camera(cam_id);
                windows_closed++;
            }
        }
        return windows_closed;
    }


};


BOOST_PYTHON_MODULE(_utrenderapi)
        {

                bp::class_<WindowBaseWrap, boost::noncopyable>("WindowBase")
                        .def("title", &WindowBase::title, &WindowBaseWrap::default_title)
                        .def("is_valid", &WindowBase::is_valid, &WindowBaseWrap::default_is_valid)
                        .def("create", &WindowBase::create, &WindowBaseWrap::default_create)
                        .def("destroy", &WindowBase::destroy, &WindowBaseWrap::default_destroy)
                        .def("pre_render", &WindowBase::pre_render, &WindowBaseWrap::default_pre_render)
                        .def("post_render", &WindowBase::post_render, &WindowBaseWrap::default_post_render)
                        .def("pre_init_gl", &WindowBase::pre_init_gl, &WindowBaseWrap::default_pre_init_gl)
                        .def("post_init_gl", &WindowBase::post_init_gl, &WindowBaseWrap::default_post_init_gl)
                        .def("on_window_size", &WindowBase::on_window_size, &WindowBaseWrap::default_on_window_size)
                        .def("on_render", &WindowBase::on_render, &WindowBaseWrap::default_on_render)
                        .def("on_window_close", &WindowBase::on_window_close, &WindowBaseWrap::default_on_window_close)
                        .def("on_keypress", &WindowBase::on_keypress, &WindowBaseWrap::default_on_keypress)
                        .def("on_cursorpos", &WindowBase::on_cursorpos, &WindowBaseWrap::default_on_cursorpos)
                ;


        }
