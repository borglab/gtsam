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
GTSAM_EXPORT std::string serializeGraph(const NonlinearFactorGraph& graph);

GTSAM_EXPORT NonlinearFactorGraph::shared_ptr deserializeGraph(const std::string& serialized_graph);

GTSAM_EXPORT std::string serializeGraphXML(const NonlinearFactorGraph& graph,
    const std::string& name = "graph");

GTSAM_EXPORT NonlinearFactorGraph::shared_ptr deserializeGraphXML(const std::string& serialized_graph,
    const std::string& name = "graph");


// Serialize/Deserialize a Values
GTSAM_EXPORT std::string serializeValues(const Values& values);

GTSAM_EXPORT Values::shared_ptr deserializeValues(const std::string& serialized_values);

GTSAM_EXPORT std::string serializeValuesXML(const Values& values, const std::string& name = "values");

GTSAM_EXPORT Values::shared_ptr deserializeValuesXML(const std::string& serialized_values,
    const std::string& name = "values");

// Serialize to/from files
// serialize functions return true if successful
// Filename arguments include path

// Serialize
GTSAM_EXPORT bool serializeGraphToFile(const NonlinearFactorGraph& graph, const std::string& fname);
GTSAM_EXPORT bool serializeGraphToXMLFile(const NonlinearFactorGraph& graph,
    const std::string& fname, const std::string& name = "graph");

GTSAM_EXPORT bool serializeValuesToFile(const Values& values, const std::string& fname);
GTSAM_EXPORT bool serializeValuesToXMLFile(const Values& values,
    const std::string& fname, const std::string& name = "values");

// Deserialize
GTSAM_EXPORT NonlinearFactorGraph::shared_ptr deserializeGraphFromFile(const std::string& fname);
GTSAM_EXPORT NonlinearFactorGraph::shared_ptr deserializeGraphFromXMLFile(const std::string& fname,
    const std::string& name = "graph");

GTSAM_EXPORT Values::shared_ptr deserializeValuesFromFile(const std::string& fname);
GTSAM_EXPORT Values::shared_ptr deserializeValuesFromXMLFile(const std::string& fname,
    const std::string& name = "values");

} // \namespace gtsam


