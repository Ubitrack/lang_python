
__author__ = 'jack'

from ubitrack.core import math as utmath
from ubitrack.core import util
from ubitrack.core import measurement
from ubitrack.facade import facade
import numpy as np
import time
import os

def setup_facade():
    "set up test fixtures"
    util.initLogging("log4cpp.conf")
    # Assumes UBITRACK_COMPONENTS_PATH is set correctly .. we don't know where components are located ..
    return facade.AdvancedFacade() 


def teardown_facade(f):
    "tear down test fixtures"
    if f is not None:
        f.clearDataflow()


@with_setup(setup_func, teardown_func)
def test_basic_facade_components():

    f = setup_facade()
    thisdir = os.path.dirname(__file__)
    f.loadDataflow(os.path.join(thisdir, "single_pushsinkpose.dfg"), True)
    
    results = []

    def cb(m):
        results.append(m)
    
    x = f.getApplicationPushSinkPose("receiver")
    if x is None:
        raise RuntimeError("Wrapping is not working properly !!!!")
    
    x.setCallback(cb)

    f.setCallbackPose("receiver", cb)
    f.startDataflow()
    
    time.sleep(3)
    
    f.stopDataflow()

    assert len(results) > 0 
    teardown_facade(f)


def test_pull_positionlist():

    f = setup_facade()
    thisdir = os.path.dirname(__file__)
    f.loadDataflow(os.path.join(thisdir, "test_positionlist.dfg"), True)
    
    x = f.getApplicationPullSinkPositionList("receiver")
    if x is None:
        raise RuntimeError("Wrapping is not working properly !!!!")
    
    f.startDataflow()
    
    mps = x.get(measurement.now())
    
    f.stopDataflow()

    ps = mps.get()
    assert len(ps) == 3 
    
    p0 = ps[0]
    assert p0[0] == 1 and p0[1] == 0 and p0[2] == 0
    teardown_facade(f)
    
def test_pullsource_pose():

    f = setup_facade()
    thisdir = os.path.dirname(__file__)
    f.loadDataflow(os.path.join(thisdir, "test_pull_source_pose.dfg"), True)
    
    x = f.getApplicationPullSourcePose("pose")
    if x is None:
        raise RuntimeError("Wrapping is not working properly !!!!")

    def pull_cb(ts):
        from ubitrack.core import math, measurement
        import numpy as np
        p = math.Pose(math.Quaternion(), np.array([1,2,3]))
        return measurement.Pose(ts, p)

    x.setCallback(pull_cb)
    
    f.startDataflow()
    
    time.sleep(3)
    
    f.stopDataflow()
    x.setCallback(None)
    teardown_facade(f)


