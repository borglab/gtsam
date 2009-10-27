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
#include "Testable.h"

namespace gtsam {

/** Chordal Bayes Net, the result of eliminating a factor graph */
class ChordalBayesNet : public Testable<ChordalBayesNet>
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

	/** print */
	void print(const std::string& s="") const;

	/** check equality */
	bool equals(const ChordalBayesNet& cbn, double tol=1e-9) const;

	/** insert: use reverse topological sort (i.e. parents last) */
	void insert(const std::string& key, ConditionalGaussian::shared_ptr node);

	/** delete */
	void erase(const std::string& key);

	/** return node with given key */
	inline ConditionalGaussian::shared_ptr get (const std::string& key) const {
		const_iterator cg = nodes.find(key); // get node
		assert( cg != nodes.end() );
		return cg->second;
	}

	inline ConditionalGaussian::shared_ptr operator[](const std::string& key) const {
		const_iterator cg = nodes.find(key); // get node
		assert( cg != nodes.end() );
		return cg->second;
	}

	/** return begin and end of the nodes. FD: breaks encapsulation? */
	typedef Nodes::const_iterator const_iterator;
	const_iterator const begin() const {return nodes.begin();}
	const_iterator const end()   const {return nodes.end();}

	/**
	 * optimize, i.e. return x = inv(R)*d
	 */
	boost::shared_ptr<VectorConfig> optimize() const;

	/** size is the number of nodes */
	size_t size() const {return nodes.size();}

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
