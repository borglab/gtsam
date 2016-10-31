/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   FactorGraph-inl.h
 * @brief  Factor Graph Base Class
 * @author Carlos Nieto
 * @author Frank Dellaert
 * @author Alireza Fathi
 * @author Michael Kaess
 * @author Richard Roberts
 */

#pragma once

#include <gtsam/inference/FactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/bind.hpp>

#include <limits>
#include <stdio.h>
#include <sstream>
#include <iostream> // for cout :-(

namespace gtsam {

  /* ************************************************************************* */
  template<class FACTOR>
  void FactorGraph<FACTOR>::print(const std::string& s, const KeyFormatter& formatter) const {
    std::cout << s << std::endl;
    std::cout << "size: " << size() << std::endl;
    for (size_t i = 0; i < factors_.size(); i++) {
      std::stringstream ss;
      ss << "factor " << i << ": ";
      if (factors_[i])
        factors_[i]->print(ss.str(), formatter);
    }
  }

  /* ************************************************************************* */
  template<class FACTOR>
  bool FactorGraph<FACTOR>::equals(const This& fg, double tol) const {
    /** check whether the two factor graphs have the same number of factors_ */
    if (factors_.size() != fg.size()) return false;

    /** check whether the factors_ are the same */
    for (size_t i = 0; i < factors_.size(); i++) {
      // TODO: Doesn't this force order of factor insertion?
      sharedFactor f1 = factors_[i], f2 = fg.factors_[i];
      if (f1 == NULL && f2 == NULL) continue;
      if (f1 == NULL || f2 == NULL) return false;
      if (!f1->equals(*f2, tol)) return false;
    }
    return true;
  }

  /* ************************************************************************* */
  template<class FACTOR>
  size_t FactorGraph<FACTOR>::nrFactors() const {
    size_t size_ = 0;
    for(const sharedFactor& factor: factors_)
      if (factor) size_++;
    return size_;
  }

  /* ************************************************************************* */
  template<class FACTOR>
  KeySet FactorGraph<FACTOR>::keys() const {
    KeySet keys;
    for(const sharedFactor& factor: this->factors_) {
      if(factor)
        keys.insert(factor->begin(), factor->end());
    }
    return keys;
  }

  /* ************************************************************************* */
  template <class FACTOR>
  KeyVector FactorGraph<FACTOR>::keyVector() const {
    KeyVector keys;
    keys.reserve(2 * size());  // guess at size
    for (const sharedFactor& factor: factors_)
      if (factor)
        keys.insert(keys.end(), factor->begin(), factor->end());
    std::sort(keys.begin(), keys.end());
    auto last = std::unique(keys.begin(), keys.end());
    keys.erase(last, keys.end());
    return keys;
  }
  
  /* ************************************************************************* */
  template<class FACTOR>
  void FactorGraph<FACTOR>::saveGraph(std::ostream &stm, const Values& values,
                         const GraphvizFormatting& formatting,
                         const KeyFormatter& keyFormatter) const
  {
    using namespace std;
    stm << "graph {\n";
    stm << "  size=\"" << formatting.figureWidthInches << "," <<
        formatting.figureHeightInches << "\";\n\n";
    
    KeySet keys = this->keys();
    
    // Local utility function to extract x and y coordinates
    struct { boost::optional<Point2> operator()(
        const Value& value, const GraphvizFormatting& graphvizFormatting)
      {
        Vector3 t;
        if (const GenericValue<Pose2>* p = dynamic_cast<const GenericValue<Pose2>*>(&value)) {
          t << p->value().x(), p->value().y(), 0;
        } else if (const GenericValue<Point2>* p = dynamic_cast<const GenericValue<Point2>*>(&value)) {
          t << p->value().x(), p->value().y(), 0;
        } else if (const GenericValue<Pose3>* p = dynamic_cast<const GenericValue<Pose3>*>(&value)) {
          t = p->value().translation();
        } else if (const GenericValue<Point3>* p = dynamic_cast<const GenericValue<Point3>*>(&value)) {
          t = p->value();
        } else {
          return boost::none;
        }
        double x, y;
        switch (graphvizFormatting.paperHorizontalAxis) {
          case GraphvizFormatting::X: x = t.x(); break;
          case GraphvizFormatting::Y: x = t.y(); break;
          case GraphvizFormatting::Z: x = t.z(); break;
          case GraphvizFormatting::NEGX: x = -t.x(); break;
          case GraphvizFormatting::NEGY: x = -t.y(); break;
          case GraphvizFormatting::NEGZ: x = -t.z(); break;
          default: throw std::runtime_error("Invalid enum value");
        }
        switch (graphvizFormatting.paperVerticalAxis) {
          case GraphvizFormatting::X: y = t.x(); break;
          case GraphvizFormatting::Y: y = t.y(); break;
          case GraphvizFormatting::Z: y = t.z(); break;
          case GraphvizFormatting::NEGX: y = -t.x(); break;
          case GraphvizFormatting::NEGY: y = -t.y(); break;
          case GraphvizFormatting::NEGZ: y = -t.z(); break;
          default: throw std::runtime_error("Invalid enum value");
        }
        return Point2(x,y);
      }} getXY;
    
    // Find bounds
    double minX = numeric_limits<double>::infinity(), maxX = -numeric_limits<double>::infinity();
    double minY = numeric_limits<double>::infinity(), maxY = -numeric_limits<double>::infinity();
    for (const Key& key : keys) {
      if (values.exists(key)) {
        boost::optional<Point2> xy = getXY(values.at(key), formatting);
        if(xy) {
          if(xy->x() < minX)
            minX = xy->x();
          if(xy->x() > maxX)
            maxX = xy->x();
          if(xy->y() < minY)
            minY = xy->y();
          if(xy->y() > maxY)
            maxY = xy->y();
        }
      }
    }
    
    // Create nodes for each variable in the graph
    for(Key key: keys){
      // Label the node with the label from the KeyFormatter
      stm << "  var" << key << "[label=\"" << keyFormatter(key) << "\"";
      if(values.exists(key)) {
        boost::optional<Point2> xy = getXY(values.at(key), formatting);
        if(xy)
          stm << ", pos=\"" << formatting.scale*(xy->x() - minX) << "," << formatting.scale*(xy->y() - minY) << "!\"";
      }
      stm << "];\n";
    }
    stm << "\n";
    
    if (formatting.mergeSimilarFactors) {
      // Remove duplicate factors
      std::set<vector<Key> > structure;
      for (const sharedFactor& factor : factors_) {
        if (factor) {
          vector<Key> factorKeys = factor->keys();
          std::sort(factorKeys.begin(), factorKeys.end());
          structure.insert(factorKeys);
        }
      }
      
      // Create factors and variable connections
      size_t i = 0;
      for(const vector<Key>& factorKeys: structure){
        // Make each factor a dot
        stm << "  factor" << i << "[label=\"\", shape=square, style=filled, fillcolor=black, width=0.3";
        {
          map<size_t, Point2>::const_iterator pos = formatting.factorPositions.find(i);
          if(pos != formatting.factorPositions.end())
            stm << ", pos=\"" << formatting.scale*(pos->second.x() - minX) << ","
                << formatting.scale*(pos->second.y() - minY) << "!\"";
        }
        stm << "];\n";
        
        // Make factor-variable connections
        for(Key key: factorKeys) {
          stm << "  var" << key << "--" << "factor" << i << ";\n";
        }
        
        ++ i;
      }
    } else {
      // Create factors and variable connections
      for(size_t i = 0; i < size(); ++i) {
        const sharedFactor& factor = at(i);
        if(formatting.plotFactorPoints) {
          const FastVector<Key>& keys = factor->keys();
          if (formatting.binaryEdges && keys.size()==2) {
            stm << "  var" << keys[0] << "--" << "var" << keys[1] << ";\n";
          } else {
            // Make each factor a dot
            stm << "  factor" << i << "[label=\"\", shape=square, style=filled, fillcolor=black, width=0.3";
            {
              map<size_t, Point2>::const_iterator pos = formatting.factorPositions.find(i);
              if(pos != formatting.factorPositions.end())
                stm << ", pos=\"" << formatting.scale*(pos->second.x() - minX) << ","
                    << formatting.scale*(pos->second.y() - minY) << "!\"";
            }
            stm << "];\n";
            
            // Make factor-variable connections
            if(formatting.connectKeysToFactor && factor) {
              for(Key key: *factor) {
                stm << "  var" << key << "--" << "factor" << i << ";\n";
              }
            }
          }
        }
        else {
          if(factor) {
            Key k;
            bool firstTime = true;
            for(Key key: *this->at(i)) {
              if(firstTime) {
                k = key;
                firstTime = false;
                continue;
              }
              stm << "  var" << key << "--" << "var" << k << ";\n";
              k = key;
            }
          }
        }
      }
    }
    
    stm << "}\n";
  }
  
  /* ************************************************************************* */
} // namespace gtsam
