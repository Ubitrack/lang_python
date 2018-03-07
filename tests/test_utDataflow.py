
__author__ = 'jack'

from ubitrack.core import math as utmath
from ubitrack.dataflow import dataflow, graph
import numpy as np
import os


# XXX this is currently not working
# def test_basic_dataflow_components():
#     "test basic dataflow components"
 
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
       
#     p1.connect(p2)
#     c1.start()
#     c.start()
#     c.stop()
#     c1.stop()
#     p1.disconnect(p2)
#     c.setEventPriority(12)
#     assert c.getEventPriority() == 12


# Graph Wrapper
# @todo 
# Needs tests for some calls to .map() or .isEqual() .. 
# they cause unicode errors and the interpreter hangs afterwards



def read_request():
    fname = os.path.join(os.path.dirname(__file__), "position_record_test.srg")
    return graph.readUTQLDocument(fname)

def read_response():
    fname = os.path.join(os.path.dirname(__file__), "position_record_test.dfg")
    return graph.readUTQLDocument(fname)


def test_keyvalueattributes():
    kva = graph.KeyValueAttributes()
    assert kva.hasAttribute("abc") == False

    v1 = "TestString"
    a1 = graph.AttributeValue(v1)
    assert a1.getText() == v1
    assert a1.isNumber() == False

    v2 = 123
    a2 = graph.AttributeValue(v2)
    assert a2.isNumber() == True
    assert a2.getNumber() == v2

    kva.setAttribute("one", a1)
    kva.setAttribute("two", a2)
    assert len(kva.map()) == 2

    assert kva.getAttribute('one').getText() == v1
    assert kva.getAttribute('two').getNumber() == v2


def test_read_utqlfile_request():
	doc = read_request()
	assert doc.isRequest() == True

def test_read_utqlfile_request():
	doc = read_response()
	assert doc.isRequest() == False

def test_inspect_request():
	doc = read_request()
	keys = doc.SubgraphById.keys()
	assert len(keys) == 2
	assert doc.hasSubgraphById('PositionTestSensor1000')
	assert doc.hasSubgraphById('Position3DRecorder1001')

	k1 = doc.getSubgraphById('PositionTestSensor1000')
	assert k1.Name == 'PositionTestSensor'
	assert k1.ID == 'PositionTestSensor1000'
	assert k1.order() == 2
	assert k1.size() == 1
	assert k1.empty() == False

	k2 = doc.getSubgraphById('Position3DRecorder1001')
	assert k2.Name == 'Position3DRecorder'
	assert k2.ID == 'Position3DRecorder1001'
	assert k2.order() == 2
	assert k2.size() == 1
	assert k2.empty() == False

	assert len(k1.Edges) == 1
	assert 'Output' in k1.Edges.keys()
	e1 = k1.getEdge('Output')
	assert e1.Name == 'Output'
	assert e1.isInput() == False
	assert e1.isOutput() == True
	assert e1.getSourceName() == 'Sensor'
	assert e1.getSourceQualifiedName() == 'node_1'
	assert e1.getTargetName() == 'Object'
	assert e1.getTargetQualifiedName() == 'node_2'

	assert len(k2.Edges) == 1
	assert 'Input' in k2.Edges.keys()
	e2 = k2.getEdge('Input')
	assert e2.Name == 'Input'
	assert e2.isInput() == True
	assert e2.isOutput() == False
	assert e2.getSourceName() == 'A'
	assert e2.getSourceQualifiedName() == ''
	assert e2.getTargetName() == 'B'
	assert e2.getTargetQualifiedName() == ''

	assert len(k1.Nodes) == 2
	assert 'Object' in k1.Nodes.keys()
	assert 'Sensor' in k1.Nodes.keys()
	n11 = k1.getNode('Object')
	assert n11.isInput() == False
	assert n11.isOutput() == True
	n12 = k1.getNode('Sensor')


	assert len(k2.Nodes) == 2
	assert 'A' in k2.Nodes.keys()
	assert 'B' in k2.Nodes.keys()
	n21 = k2.getNode('A')
	n22 = k2.getNode('B')


def test_inspect_response():
	doc = read_response()
	keys = doc.SubgraphById.keys()
	assert len(keys) == 2
	assert doc.hasSubgraphById('pattern_1')
	assert doc.hasSubgraphById('pattern_2')

	k1 = doc.getSubgraphById('pattern_1')
	assert k1.Name == 'PositionTestSensor'
	assert k1.ID == 'pattern_1'
	assert k1.order() == 2
	assert k1.size() == 1
	assert k1.empty() == False

	k2 = doc.getSubgraphById('pattern_2')
	assert k2.Name == 'Position3DRecorder'
	assert k2.ID == 'pattern_2'
	assert k2.order() == 2
	assert k2.size() == 1
	assert k2.empty() == False

	assert len(k1.Edges) == 1
	assert 'Output' in k1.Edges.keys()
	e1 = k1.getEdge('Output')
	assert e1.Name == 'Output'
	assert e1.isInput() == False
	assert e1.isOutput() == True
	assert e1.getSourceName() == 'Sensor'
	assert e1.getSourceQualifiedName() == 'node_1'
	assert e1.getTargetName() == 'Object'
	assert e1.getTargetQualifiedName() == 'node_2'

	assert len(k2.Edges) == 1
	assert 'Input' in k2.Edges.keys()
	e2 = k2.getEdge('Input')
	assert e2.Name == 'Input'
	assert e2.isInput() == True
	assert e2.isOutput() == False
	assert e2.getSourceName() == 'A'
	assert e2.getSourceQualifiedName() == 'node_1'
	assert e2.getTargetName() == 'B'
	assert e2.getTargetQualifiedName() == 'node_2'

	assert len(k1.Nodes) == 2
	assert 'Object' in k1.Nodes.keys()
	assert 'Sensor' in k1.Nodes.keys()
	n11 = k1.getNode('Object')
	assert n11.isInput() == False
	assert n11.isOutput() == True
	n12 = k1.getNode('Sensor')


	assert len(k2.Nodes) == 2
	assert 'A' in k2.Nodes.keys()
	assert 'B' in k2.Nodes.keys()
	n21 = k2.getNode('A')
	n22 = k2.getNode('B')


