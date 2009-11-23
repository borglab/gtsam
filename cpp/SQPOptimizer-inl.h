/*
 * @file SQPOptimizer-inl.h
 * @brief Implementation of the SQP Optimizer
 * @author Alex Cunningham
 */

#pragma once

#include "SQPOptimizer.h"

using namespace std;
namespace gtsam {

/* **************************************************************** */
template <class G, class C>
SQPOptimizer<G,C>::SQPOptimizer(const G& graph, const Ordering& ordering,
		shared_config config)
: graph_(&graph), ordering_(&ordering), config_(config)
{
	// TODO: assign a value to the lagrange config

}

/* **************************************************************** */
template <class G, class C>
SQPOptimizer<G,C>::SQPOptimizer(const G& graph, const Ordering& ordering,
		shared_config config, shared_vconfig lagrange)
: graph_(&graph), ordering_(&ordering), config_(config), lagrange_config_(lagrange)
{

}

}
