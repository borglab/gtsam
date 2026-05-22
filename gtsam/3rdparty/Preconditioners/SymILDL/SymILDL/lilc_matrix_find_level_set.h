// -*- mode: c++ -*-
#ifndef _LILC_MATRIX_FIND_LEVEL_SET_H_
#define _LILC_MATRIX_FIND_LEVEL_SET_H_

template<class el_type>
inline bool lilc_matrix<el_type> :: find_level_set(vector<int>& lvl_set, vector<bool>& visited) {
	vector<int> new_set;
	for (idx_it node_it = lvl_set.begin(); node_it != lvl_set.end(); node_it++) {

		for (idx_it it = list[*node_it].begin(); it != list[*node_it].end(); it++) {
			if (!visited[*it]) {
				visited[*it] = true;
				new_set.push_back(*it);
			}
		}
		
		for (idx_it it = m_idx[*node_it].begin(); it != m_idx[*node_it].end(); it++) {
			if (!visited[*it]) {
				visited[*it] = true;
				new_set.push_back(*it);
			}
		}
	}
	
	if (new_set.empty()) return false;
	
	lvl_set.swap(new_set);
	return true;
}

#endif