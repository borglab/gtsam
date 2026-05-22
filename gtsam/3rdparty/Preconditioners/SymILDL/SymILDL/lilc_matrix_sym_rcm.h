// -*- mode: c++ -*-
#ifndef _LILC_MATRIX_SYM_RCM_H_
#define _LILC_MATRIX_SYM_RCM_H_

namespace {
/*! \brief Functor for comparing elements by degree (in increasing order) instead of by index.
	\param A a pointer to the matrix being reordered.
*/
template <class el_type>
struct by_degree {
	lilc_matrix<el_type>* A;
	by_degree(lilc_matrix<el_type>* mat) : A(mat) {}
	bool operator()(int const &a, int const &b) const { 
		int deg_a = A->list[a].size() + A->m_idx[a].size();
		int deg_b = A->list[b].size() + A->m_idx[b].size();
		
		if (A->m_idx[a].size() > 0 && A->m_idx[a][0] == a) deg_a--;
		if (A->m_idx[b].size() > 0 && A->m_idx[b][0] == b) deg_b--;
		
		if (deg_a == deg_b) return a > b;
		return deg_a < deg_b;
	}
};
}

template<class el_type> 
inline void lilc_matrix<el_type> :: sym_rcm(vector<int>& perm) {
	int i, s;
	vector<bool> visited(m_n_cols, false);
	vector<int> lvl_set;
	for (i = 0; i < m_n_cols; i++) {
		if (visited[i]) continue;
		
		lvl_set.clear();
		s = i;
		find_root(s);
		lvl_set.push_back(s);
		perm.push_back(s);
		
		by_degree<el_type> sorter(this);
		
		visited[s] = true;
		while (find_level_set(lvl_set, visited)) {
			sort(lvl_set.begin(), lvl_set.end(), sorter);
			perm.insert( perm.end(), lvl_set.begin(), lvl_set.end() );	
		}
	}
	
	reverse(perm.begin(), perm.end());
}

#endif