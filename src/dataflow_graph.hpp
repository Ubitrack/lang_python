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
#include <ubitrack_python/pystreambuf.h>

#include <boost/shared_ptr.hpp>
#include <tuple>
#include <map>
#include <vector>
#include <iostream>
#include <fstream>

#include <utGraph/UTQLDocument.h>
#include <utGraph/UTQLReader.h>
#include <utGraph/Graph.h>



// type declaration shortcut
typedef Ubitrack::Graph::Graph< Ubitrack::Graph::UTQLNode, Ubitrack::Graph::UTQLEdge > UTQLGraph;

boost::shared_ptr< Ubitrack::Graph::UTQLDocument > UTQLReader_processInput( std::istream& stream ) {
	// then load utqlfile and test basic graph wrapping
	boost::shared_ptr< Ubitrack::Graph::UTQLDocument > doc;
	doc = Ubitrack::Graph::UTQLReader::processInput( stream );
	return doc;
	}

boost::shared_ptr< Ubitrack::Graph::UTQLDocument > UTQLReader_processInput( const std::string& filename ) {
	std::ifstream stream( filename.c_str() );
	if ( !stream.good() )
		throw std::runtime_error( "Could not open file " + filename + " for reading" );
	return UTQLReader_processInput(stream);
	}


UTQLGraph::NodePtr get_SourceNode_from_weak_ptr(const UTQLGraph::Edge& e) {
	UTQLGraph::NodePtr sp = e.m_Source.lock ();
	return sp;
}

UTQLGraph::NodePtr get_TargetNode_from_weak_ptr(const UTQLGraph::Edge& e) {
	UTQLGraph::NodePtr sp = e.m_Target.lock ();
	return sp;
}

std::string get_SourceNodeName_from_weak_ptr(const UTQLGraph::Edge& e) {
	UTQLGraph::NodePtr sp = e.m_Source.lock ();
	std::string ret = "";
	if (sp) {
		ret = sp->m_Name;
	}
	return ret;
}

std::string get_TargetNodeName_from_weak_ptr(const UTQLGraph::Edge& e) {
	UTQLGraph::NodePtr sp = e.m_Target.lock ();
	std::string ret = "";
	if (sp) {
		ret = sp->m_Name;
	}
	return ret;
}

std::string get_SourceNodeQualifiedName_from_weak_ptr(const UTQLGraph::Edge& e) {
	UTQLGraph::NodePtr sp = e.m_Source.lock ();
	std::string ret = "";
	if (sp) {
		ret = sp->m_QualifiedName;
	}
	return ret;
}

std::string get_TargetNodeQualifiedName_from_weak_ptr(const UTQLGraph::Edge& e) {
	UTQLGraph::NodePtr sp = e.m_Target.lock ();
	std::string ret = "";
	if (sp) {
		ret = sp->m_QualifiedName;
	}
	return ret;
}

py::list get_InEdgeList_from_weak_ptr(const UTQLGraph::Node& n) {
	py::list edges;
	for (UTQLGraph::Node::EdgeList::const_iterator it = n.m_InEdges.begin(); it != n.m_InEdges.end(); it++) {
		UTQLGraph::EdgePtr ep = it->lock ();
		if (ep) {
			edges.append(*(ep.get()));
		}
	}
	return edges;
}

py::list get_OutEdgeList_from_weak_ptr(const UTQLGraph::Node& n) {
	py::list edges;
	for (UTQLGraph::Node::EdgeList::const_iterator it = n.m_OutEdges.begin(); it != n.m_OutEdges.end(); it++) {
		UTQLGraph::EdgePtr ep = it->lock ();
		if (ep) {
			edges.append(*(ep.get()));
		}
	}
	return edges;
}

py::object get_xml_from_attrvalue_node(const Ubitrack::Graph::AttributeValue& av) {
	const TiXmlElement* np = av.getXML();
	if (np) {
		TiXmlPrinter printer;
		printer.SetIndent( "  " );
		np->Accept( &printer );

		std::string data = "<root xmlns:xsi=\"http://www.xml.org/xsi\" >";
		data += std::string(printer.CStr());
		data += "</root>";

		return py::bytes(data);
	} else {
		return py::none();
	}

}



void bind_utDataflowGraph(py::module& m)
{


	py::class_< Ubitrack::Graph::AttributeValue, boost::shared_ptr< Ubitrack::Graph::AttributeValue > >(m, "AttributeValue")
		.def(py::init<const std::string&>())
		.def(py::init<double>())
		.def("getText", &Ubitrack::Graph::AttributeValue::getText)
		.def("getNumber", &Ubitrack::Graph::AttributeValue::getNumber)
		.def("isNumber", &Ubitrack::Graph::AttributeValue::isNumber)
		.def("isEmpty", &Ubitrack::Graph::AttributeValue::isEmpty)
		.def(py::self == py::self)
		.def("getXML", &get_xml_from_attrvalue_node, py::return_value_policy::reference_internal)
		;

	py::class_< Ubitrack::Graph::KeyValueAttributes, boost::shared_ptr< Ubitrack::Graph::KeyValueAttributes > >gr_kva(m, "KeyValueAttributes");
	gr_kva
		.def(py::init<>())
		.def("getAttribute", &Ubitrack::Graph::KeyValueAttributes::getAttribute)
		.def("setAttribute", &Ubitrack::Graph::KeyValueAttributes::setAttribute)
		.def("hasAttribute", &Ubitrack::Graph::KeyValueAttributes::hasAttribute)

		.def("getAttributeString", &Ubitrack::Graph::KeyValueAttributes::getAttributeString)

		.def("isEqual", &Ubitrack::Graph::KeyValueAttributes::isEqual)
		//.def("mergeAttributes", &Ubitrack::Graph::KeyValueAttributes::mergeAttributes)
		//.def("swap", &Ubitrack::Graph::KeyValueAttributes::swap)
		.def("map", &Ubitrack::Graph::KeyValueAttributes::map, py::return_value_policy::reference_internal)
		;


	py::class_< Ubitrack::Graph::InOutAttribute, boost::shared_ptr< Ubitrack::Graph::InOutAttribute > >gr_ioa(m, "InOutAttribute", gr_kva);
	gr_ioa
		.def("isInput", &Ubitrack::Graph::InOutAttribute::isInput)
		.def("isOutput", &Ubitrack::Graph::InOutAttribute::isOutput)
		;

    py::enum_<Ubitrack::Graph::InOutAttribute::Tag>(gr_ioa, "Tag")
	    .value("Input", Ubitrack::Graph::InOutAttribute::Input)
	    .value("Output", Ubitrack::Graph::InOutAttribute::Output)
	    ;


	py::class_< UTQLGraph::Node, boost::shared_ptr< UTQLGraph::Node > >(m, "UTQLGraphNode", gr_ioa)
		.def(py::init<const std::string&, const Ubitrack::Graph::UTQLNode&>())
		.def_readonly("Name", &UTQLGraph::Node::m_Name)
		.def("getInEdges", &get_InEdgeList_from_weak_ptr)
		.def("getOutEdges", &get_OutEdgeList_from_weak_ptr)
		;

	py::class_< UTQLGraph::Edge, boost::shared_ptr< UTQLGraph::Edge > >(m, "UTQLGraphEdge", gr_ioa)
		// .def(py::init<const std::string&, Ubitrack::Graph::GraphEdge< Ubitrack::Graph::UTQLNode, Ubitrack::Graph::UTQLEdge >::WeakNodePtr, Ubitrack::Graph::GraphEdge< Ubitrack::Graph::UTQLNode, Ubitrack::Graph::UTQLEdge >::WeakNodePtr >())
		.def_readonly("Name", &UTQLGraph::Edge::m_Name)
		.def("getSource", &get_SourceNode_from_weak_ptr)
		.def("getTarget", &get_TargetNode_from_weak_ptr)
		.def("getSourceName", &get_SourceNodeName_from_weak_ptr)
		.def("getTargetName", &get_TargetNodeName_from_weak_ptr)
		.def("getSourceQualifiedName", &get_SourceNodeQualifiedName_from_weak_ptr)
		.def("getTargetQualifiedName", &get_TargetNodeQualifiedName_from_weak_ptr)
		.def_readonly("EdgeReference", &Ubitrack::Graph::UTQLEdge::m_EdgeReference)
		;


	py::class_< UTQLGraph, boost::shared_ptr< UTQLGraph > >gr_utqlgr(m, "UTQLGraph");
	gr_utqlgr
		.def("addNode", &UTQLGraph::addNode)
		.def("hasNode", &UTQLGraph::hasNode)
		.def("getNode", &UTQLGraph::getNode)

		.def("removeNode", (void (UTQLGraph::*)(const std::string&))&UTQLGraph::removeNode)
		.def("removeNode", (void (UTQLGraph::*)(UTQLGraph::NodePtr))&UTQLGraph::removeNode)
		.def("addEdge", (UTQLGraph::EdgePtr (UTQLGraph::*)(const std::string&,const std::string&,const std::string&,const UTQLGraph::GraphEdgeAttributes&))&UTQLGraph::addEdge, py::return_value_policy::reference_internal)
		.def("addEdge", (UTQLGraph::EdgePtr (UTQLGraph::*)(const std::string&,UTQLGraph::NodePtr,UTQLGraph::NodePtr,const UTQLGraph::GraphEdgeAttributes&))&UTQLGraph::addEdge, py::return_value_policy::reference_internal)
		.def("hasEdge", &UTQLGraph::hasEdge)
		.def("getEdge", &UTQLGraph::getEdge)

		.def("removeEdge", (void (UTQLGraph::*)(const std::string&))&UTQLGraph::removeEdge)
		.def("removeEdge", (void (UTQLGraph::*)(UTQLGraph::EdgePtr))&UTQLGraph::removeEdge)
		.def("size", &UTQLGraph::size)
		.def("order", &UTQLGraph::order)
		.def("empty", &UTQLGraph::empty)
		.def("null", &UTQLGraph::null)

		.def_readonly("Nodes", &UTQLGraph::m_Nodes)
		.def_readonly("Edges", &UTQLGraph::m_Edges)
		;

	py::class_< Ubitrack::Graph::UTQLDocument, boost::shared_ptr< Ubitrack::Graph::UTQLDocument > >(m, "UTQLDocument")
		.def(py::init<bool>())
		.def("addSubgraph", &Ubitrack::Graph::UTQLDocument::addSubgraph)
		.def("hasSubgraphById", &Ubitrack::Graph::UTQLDocument::hasSubgraphById)
		.def("getSubgraphById", &Ubitrack::Graph::UTQLDocument::getSubgraphById, py::return_value_policy::reference_internal)
		.def("removeSubgraphById", &Ubitrack::Graph::UTQLDocument::removeSubgraphById)
		.def("exportDot", &Ubitrack::Graph::UTQLDocument::exportDot)
		.def("isRequest", &Ubitrack::Graph::UTQLDocument::isRequest)
		// properties
		.def_readonly("SubgraphById", &Ubitrack::Graph::UTQLDocument::m_SubgraphById)
		// not automatically available due to the use of std::multimap
		// .def_readonly("SubgraphByName", &Ubitrack::Graph::UTQLDocument::m_SubgraphByName)
		.def_readonly("Subgraphs", &Ubitrack::Graph::UTQLDocument::m_Subgraphs)
		;

	py::class_< Ubitrack::Graph::UTQLSubgraph, boost::shared_ptr< Ubitrack::Graph::UTQLSubgraph > >(m, "UTQLSubgraph", gr_utqlgr)
		.def(py::init<std::string, std::string>())
		.def_readonly("ID", &Ubitrack::Graph::UTQLSubgraph::m_ID)
		.def_readonly("Name", &Ubitrack::Graph::UTQLSubgraph::m_Name)
		// these require much more wrapping .. but might be worthwile to investigate ..
		// .def_readonly("onlyBestEdgeMatch", &Ubitrack::Graph::UTQLSubgraph::m_onlyBestEdgeMatch)
		// .def_readonly("bestMatchExpression", &Ubitrack::Graph::UTQLSubgraph::m_bestMatchExpression)
		.def_readonly("DataflowConfiguration", &Ubitrack::Graph::UTQLSubgraph::m_DataflowConfiguration)
		.def_readonly("DataflowAttributes", &Ubitrack::Graph::UTQLSubgraph::m_DataflowAttributes)
		.def_readonly("DataflowClass", &Ubitrack::Graph::UTQLSubgraph::m_DataflowClass)
		;


	py::class_< Ubitrack::Graph::UTQLEdge, boost::shared_ptr< Ubitrack::Graph::UTQLEdge > >(m, "UTQLEdge")
		.def(py::init< Ubitrack::Graph::InOutAttribute::Tag>())
		.def_readonly("EdgeReference", &Ubitrack::Graph::UTQLEdge::m_EdgeReference)
		;

	py::class_< Ubitrack::Graph::UTQLNode, boost::shared_ptr< Ubitrack::Graph::UTQLNode > >(m, "UTQLNode")
		.def(py::init< Ubitrack::Graph::InOutAttribute::Tag>())
		.def_readonly("QualifiedName", &Ubitrack::Graph::UTQLNode::m_QualifiedName)
		;

	py::class_< Ubitrack::Graph::EdgeReference, boost::shared_ptr< Ubitrack::Graph::EdgeReference > >(m, "EdgeReference")
		.def(py::init<const std::string&, const std::string&>())
		.def("getSubgraphId", &Ubitrack::Graph::EdgeReference::getSubgraphId, py::return_value_policy::reference_internal)
		.def("getEdgeName", &Ubitrack::Graph::EdgeReference::getEdgeName, py::return_value_policy::reference_internal)
		.def("empty", &Ubitrack::Graph::EdgeReference::empty)
		;

	m.def("readUTQLDocument", (boost::shared_ptr< Ubitrack::Graph::UTQLDocument > (*)(std::istream&))&UTQLReader_processInput);
	m.def("readUTQLDocument", (boost::shared_ptr< Ubitrack::Graph::UTQLDocument > (*)(const std::string&))&UTQLReader_processInput);

}