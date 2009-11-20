/**
 * @file    VectorConfig.h
 * @brief   Factor Graph Configuration
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include <map>
#include <boost/serialization/map.hpp>

#include "Testable.h"
#include "Vector.h"

namespace gtsam {
	
  /** Factor Graph Configuration */
  class VectorConfig : public Testable<VectorConfig> {

  protected:
    /** Map from string indices to values */
    std::map<std::string, Vector> values;

  public:
    typedef std::map<std::string, Vector>::iterator iterator;
    typedef std::map<std::string, Vector>::const_iterator const_iterator;

    VectorConfig() {}
    VectorConfig(const VectorConfig& cfg_in): values(cfg_in.values) {}
    
    virtual ~VectorConfig() {}

    /** return all the nodes in the graph **/
    std::vector<std::string> get_names() const {
      std::vector<std::string> names;
      for(const_iterator it=values.begin(); it!=values.end(); it++)
        names.push_back(it->first);
      return names;
    }

    /** Insert a value into the configuration with a given index */
    VectorConfig& insert(const std::string& name, const Vector& val) {
      values.insert(std::make_pair(name,val));
      return *this;
    }

    /**
     * Add a delta config, needed for use in NonlinearOptimizer
     * For VectorConfig, this is just addition.
     */
    VectorConfig exmap(const VectorConfig & delta) const;

    /** Scales the configuration by a gain */
    VectorConfig scale(double gain);
 
    const_iterator begin() const {return values.begin();}
    const_iterator end()   const {return values.end();}

    /** get a vector in the configuration by name */
    Vector get(const std::string& name) const;

    /** operator[] syntax for get */
    inline Vector operator[](const std::string& name) const { return get(name); }

    bool contains(const std::string& name) const {
      const_iterator it = values.find(name);
      if (it==values.end())
        return false;
      return true;
    }

    /** size of the configurations */
    size_t size() const {
      return values.size();
    }

    /** print */
    void print(const std::string& name = "") const;

    /** equals, for unit testing */
    bool equals(const VectorConfig& expected, double tol=1e-9) const;

    void clear() {values.clear();}
    
    
  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
      void serialize(Archive & ar, const unsigned int version)
    {
      ar & BOOST_SERIALIZATION_NVP(values);
    }
  };
}
