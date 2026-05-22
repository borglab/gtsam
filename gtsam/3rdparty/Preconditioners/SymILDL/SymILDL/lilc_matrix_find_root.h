// -*- mode: c++ -*-
#ifndef _LILC_MATRIX_FIND_ROOT_H_
#define _LILC_MATRIX_FIND_ROOT_H_

template<class el_type> 
inline void lilc_matrix<el_type> :: find_root(int& s) {
	vector<bool> visited(m_n_cols, false);
	vector<int> lvl_set;
	int ls_max = 0, ls = 0;
	
	while (true) {
		lvl_set.clear();
		std::fill(visited.begin(), visited.end(), false);
		ls = 0;
		
		lvl_set.push_back(s);
		visited[s] = true;
		while (find_level_set(lvl_set, visited))
			ls++;

		if (ls > ls_max) {
			ls_max = ls;
			int deg, min_deg = m_n_cols;
			for (idx_it it = lvl_set.begin(); it != lvl_set.end(); it++) {
				deg = list[*it].size() + m_idx[*it].size();
				if (m_idx[*it].size() > 0 && m_idx[*it][0] == *it) deg--;
				if (deg < min_deg) { //should consider tie breaking by index later if needed.
					min_deg = deg;
					s = *it;
				}
			}
		} else {
			break;
		}
	}
}

#endif