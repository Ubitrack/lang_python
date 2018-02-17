
__author__ = 'jack'

from nose import with_setup

from ubitrack.core import math as utmath
from ubitrack.dataflow import dataflow
import numpy as np

def setup_func():
    "set up test fixtures"
    pass

def teardown_func():
    "tear down test fixtures"
    pass



# @with_setup(setup_func, teardown_func)
# def test_basic_dataflow_components():
#     "test basic dataflow components"
#  
#     # abstract classes for now
#     c = dataflow.Component("test1")
#     assert c.getName() == "test1"
#     # XXX adding ports results in segfaults cause by double freeing a pointer
#     # eventually, Component/Port will become abstract classes, that cannot be instantiated 
#     # from within python .. also makes sense somehow..
#     p1 = dataflow.Port("P1", c)
#     assert p1.fullName() == "test1:P1"
#     c1 = dataflow.Component("test2")
#     p2 = dataflow.Port("P2", c1)
#     assert p2.fullName() == "test2:P2"
#        
#     p1.connect(p2)
#     c1.start()
#     c.start()
#     c.stop()
#     c1.stop()
#     p1.disconnect(p2)
#     c.setEventPriority(12)
#     assert c.getEventPriority() == 12
