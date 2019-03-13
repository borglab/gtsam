/**
 * @file serialization.h
 *
 * @brief Global functions for performing serialization, designed for use with matlab
 *
 * @date Jun 12, 2013
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

// Serialize/Deserialize a NonlinearFactorGraph
std::string serializeGraph(const NonlinearFactorGraph& graph);

NonlinearFactorGraph::shared_ptr deserializeGraph(const std::string& serialized_graph);

std::string serializeGraphXML(const NonlinearFactorGraph& graph,
    const std::string& name = "graph");

NonlinearFactorGraph::shared_ptr deserializeGraphXML(const std::string& serialized_graph,
    const std::string& name = "graph");


// Serialize/Deserialize a Values
std::string serializeValues(const Values& values);

Values::shared_ptr deserializeValues(const std::string& serialized_values);

std::string serializeValuesXML(const Values& values, const std::string& name = "values");

Values::shared_ptr deserializeValuesXML(const std::string& serialized_values,
    const std::string& name = "values");

// Serialize to/from files
// serialize functions return true if successful
// Filename arguments include path

// Serialize
bool serializeGraphToFile(const NonlinearFactorGraph& graph, const std::string& fname);
bool serializeGraphToXMLFile(const NonlinearFactorGraph& graph,
    const std::string& fname, const std::string& name = "graph");

bool serializeValuesToFile(const Values& values, const std::string& fname);
bool serializeValuesToXMLFile(const Values& values,
    const std::string& fname, const std::string& name = "values");

// Deserialize
NonlinearFactorGraph::shared_ptr deserializeGraphFromFile(const std::string& fname);
NonlinearFactorGraph::shared_ptr deserializeGraphFromXMLFile(const std::string& fname,
    const std::string& name = "graph");

Values::shared_ptr deserializeValuesFromFile(const std::string& fname);
Values::shared_ptr deserializeValuesFromXMLFile(const std::string& fname,
    const std::string& name = "values");

} // \namespace gtsam


