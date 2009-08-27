/**
 * @file    ChordalBayesNet.h
 * @brief   Chordal Bayes Net, the result of eliminating a factor graph
 * @brief   ChordalBayesNet
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <list>
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>

#include "ConditionalGaussian.h"
#include "FGConfig.h"

namespace gtsam {

/** Chordal Bayes Net, the result of eliminating a factor graph */
class ChordalBayesNet
{
public:
	typedef boost::shared_ptr<ChordalBayesNet> shared_ptr;

protected:

	/** nodes keys stored in topological sort order, i.e. from parents to children */
	std::list<std::string> keys;

	/** nodes stored on key */
	typedef std::map<std::string,ConditionalGaussian::shared_ptr> Nodes;
	Nodes nodes;

	typedef Nodes::iterator iterator;

public:

	/** Construct an empty net */
	ChordalBayesNet() {}

	/** Copy Constructor */
	ChordalBayesNet(const ChordalBayesNet& cbn_in) : keys(cbn_in.keys), nodes(cbn_in.nodes) {}

	/** Destructor */
	virtual ~ChordalBayesNet() {}

	/** insert: use reverse topological sort (i.e. parents last) */
	void insert(const std::string& key, ConditionalGaussian::shared_ptr node);

	/** delete */
	void erase(const std::string& key);

	/** return node with given key */
	inline ConditionalGaussian::shared_ptr get       (const std::string& key) { return nodes[key];}
	inline ConditionalGaussian::shared_ptr operator[](const std::string& key) { return nodes[key];}

	/** return begin and end of the nodes. FD: breaks encapsulation? */
	typedef Nodes::const_iterator const_iterator;
	const_iterator const begin() const {return nodes.begin();}
	const_iterator const end()   const {return nodes.end();}

	/** optimize */
	boost::shared_ptr<FGConfig> optimize() const;
	boost::shared_ptr<FGConfig> optimize(const boost::shared_ptr<FGConfig> &c) const;

	/** print */
	void print() const;

	/** check equality */
	bool equals(const ChordalBayesNet& cbn) const;

	/**
	 * Return (dense) upper-triangular matrix representation
	 */
	std::pair<Matrix,Vector> matrix() const;

private:
	/** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & BOOST_SERIALIZATION_NVP(keys);
		ar & BOOST_SERIALIZATION_NVP(nodes);
	}

};

} /// namespace gtsam
