/**
 * @file    FGConfig.h
 * @brief   Factor Graph Configuration
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include <map>
#include <boost/serialization/map.hpp>

#include "Value.h"

namespace gtsam {
	
  /** Factor Graph Configuration */
  class FGConfig {

  protected:
    /** Map from string indices to values */
    std::map<std::string, Vector> values;

  public:
    typedef std::map<std::string, Vector>::iterator iterator;
    typedef std::map<std::string, Vector>::const_iterator const_iterator;

    FGConfig() {};
    FGConfig(const FGConfig& cfg_in) : values(cfg_in.values){};
    
    virtual ~FGConfig() {};

    /** return all the nodes in the graph **/
    std::vector<std::string> get_names() const {
      std::vector<std::string> names;
      for(const_iterator it=values.begin(); it!=values.end(); it++)
        names.push_back(it->first);
      return names;
    }

    /** Insert a value into the configuration with a given index */
    FGConfig& insert(const std::string& name, const Vector& val) {
      values.insert(std::make_pair(name,val));
      return *this;
    }

    /**
     * add a delta config, should apply exponential map more generally
     */
    virtual void operator+=(const FGConfig & delta);
    virtual FGConfig operator+(const FGConfig & delta) const;
 
    const_iterator begin() {return values.begin();}
    const_iterator end()   {return values.end();}

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
    bool equals(const FGConfig& expected, double tol=1e-6) const;

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
