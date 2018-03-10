
__author__ = 'jack'

from ubitrack.core import math as utmath
from ubitrack.core import util
from ubitrack.core import measurement
from ubitrack import facade
import numpy as np
import time
import os

logging_initialized = False

def setup_facade():
    print("set up AdvancedFacade")
    global logging_initialized
    if not logging_initialized:
        util.initLogging("log4cpp.conf")
        logging_initialized = True

    if not "UBITRACK_COMPONENTS_PATH" in os.environ:
        print("Missing environment variable: UBITRACK_COMPONENTS_PATH - tests are likely to fail")
        return facade.AdvancedFacade() 
    else:
        return facade.AdvancedFacade(os.environ['UBITRACK_COMPONENTS_PATH']) 


def teardown_facade(f):
    print("tear down AdvancedFacade")
    if f is not None:
        f.clearDataflow()
        f.killEverything()
    print("done")


def test_basic_facade_components_direct_callback():
    print("test_basic_facade_components_direct_callback")

    f = setup_facade()
    thisdir = os.path.dirname(__file__)
    f.loadDataflow(os.path.join(thisdir, "single_pushsinkpose.dfg"), True)
    
    results = []

    def cb(m):
        results.append(m)

    print("set callback pose")
    f.setCallbackPose("receiver", cb)
    print("start dataflow")
    f.startDataflow()

    print("wait")
    time.sleep(3)

    print("stop dataflow")
    f.stopDataflow()
    f.setCallbackPose("receiver", None)

    print("assert results")
    assert len(results) > 0
    print(results[0])
    results.clear()

    teardown_facade(f)
    f = None


def test_basic_facade_components_pushsink_object():
    print("test_basic_facade_components_pushsink_object")

    f = setup_facade()
    thisdir = os.path.dirname(__file__)
    f.loadDataflow(os.path.join(thisdir, "single_pushsinkpose.dfg"), True)

    results = []

    def cb(m):
        results.append(m)

    print("get pushsink")
    x = f.getApplicationPushSinkPose("receiver")
    assert x is not None

    print("set callback pose")
    x.setCallback(cb)

    print("start dataflow")
    f.startDataflow()

    print("wait")
    time.sleep(3)

    print("stop dataflow")
    f.stopDataflow()

    x.setCallback(None)
    # XXX need to deallocate pushsink otherwise we'll segfault on program exit
    x = None

    print("assert results")
    assert len(results) > 0
    print(results[0])
    results.clear()

    time.sleep(0.3)

    teardown_facade(f)
    f = None



def test_pull_positionlist():
    print("test_pull_positionlist")

    f = setup_facade()
    thisdir = os.path.dirname(__file__)
    f.loadDataflow(os.path.join(thisdir, "test_positionlist.dfg"), True)
    
    x = f.getApplicationPullSinkPositionList("receiver")
    assert x is not None

    f.startDataflow()
    
    mps = x.get(measurement.now())
    
    f.stopDataflow()
    # XXX need to deallocate pullsink otherwise we'll segfault on program exit
    x = None

    ps = mps.get()
    assert len(ps) == 3
    
    p0 = ps[0]
    assert p0[0] == 1 and p0[1] == 0 and p0[2] == 0

    time.sleep(0.3)
    teardown_facade(f)
    f = None



def test_pullsource_pose():
    print("test_pullsource_pose")
    f = setup_facade()
    thisdir = os.path.dirname(__file__)
    f.loadDataflow(os.path.join(thisdir, "test_pull_source_pose.dfg"), True)
    
    x = f.getApplicationPullSourcePose("pose")
    assert x is not None

    def pull_cb(ts):
        from ubitrack.core import math, measurement
        import numpy as np
        p = math.Pose(math.Quaternion(), np.array([1.,2.,3.], dtype=np.double))
        return measurement.Pose(ts, p)

    x.setCallback(pull_cb)
    f.startDataflow()
    
    time.sleep(3)
    
    f.stopDataflow()
    x.setCallback(None)
    # XXX need to deallocate pullsource otherwise we'll segfault on program exit
    x = None

    time.sleep(0.3)
    teardown_facade(f)
    f = None


