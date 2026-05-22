using std::abs;

/*!	There are four parts to this pivoting algorithm.
	For A, due to storing only the lower half, there are three steps to performing a symmetric permutation:
		-#	A(k, 1:k) must be swapped with A(r, 1:k) (row-row swap).
		-#	A(k:r, k) must be swapped with A(r, k:r) (row-column swap).
		-#	A(k:r, k) must be swapped with A(k:r, r) (column-column swap).
    The steps above are implemented in the pivotA function.
	
	For L, since column k and r are not yet formed, there is only one step (a row permutation):
		-#	L(k, 1:k) must be swapped with L(r, 1:k) (row-row swap).
*/
template <class el_type>
inline void lilc_matrix<el_type> :: pivot(swap_struct<el_type>& s, vector<bool>& in_set, lilc_matrix<el_type>& L, const int& k, const int& r) {	
	//initialize temp variables
	int i, j, idx, offset;

	//----------pivot A ----------//
	this->pivotA(s, in_set, k, r);
	//--------end pivot A---------//
	
	//----------pivot L ----------//
	s.swap_clear();
    
	// -------------------- (1) for L ------------------------//
	//push back pointers to L(k, i)
	for (idx_it it = L.list[k].begin(); it != L.list[k].end(); it++)
	{
		for (i = L.col_first[*it]; i < (int) L.m_idx[*it].size(); i++) {
			if (L.m_idx[*it][i] == k) {
				s.swapr.push_back(L.m_idx[*it].begin() + i);
				break;
			}
		}
	}
	
	//push back pointers to L(r, i)
	for (idx_it it = L.list[r].begin(); it != L.list[r].end(); it++) {
		for (i = L.col_first[*it]; i < (int) L.m_idx[*it].size(); i++) {
			if (L.m_idx[*it][i] == r) {
				s.swapk.push_back(L.m_idx[*it].begin() + i);
				break;
			}
		}
	}
	
	//swap rows k and r
	for (typename vector<idx_it>::iterator it = s.swapk.begin(); it != s.swapk.end(); it++) {
		**it = k;
	}
	for (typename vector<idx_it>::iterator it = s.swapr.begin(); it != s.swapr.end(); it++) {
		**it = r;
	}

	//row swap on row non-zero indices stored in L.list
	L.list[k].swap(L.list[r]);

	//--------end pivot L---------//
}

/*!	There are three parts to this pivoting algorithm.
	For A, due to storing only the lower half, there are three steps to performing a symmetric permutation:
		-#	A(k, 1:k) must be swapped with A(r, 1:k) (row-row swap).
		-#	A(k:r, k) must be swapped with A(r, k:r) (row-column swap).
		-#	A(k:r, k) must be swapped with A(k:r, r) (column-column swap).
*/
template <class el_type>
inline void lilc_matrix<el_type> :: pivotA(swap_struct<el_type>& s, vector<bool>& in_set, const int& k, const int& r) {
	assert(k < r); // this algorithm implicitly assumes k < r
	
	//initialize temp variables
	std::pair<idx_it, elt_it> its_k, its_r;
	int i, j, idx, offset;
	
	//----------- clear out old variables from last pivot -------------- //
	//for vectors of primitive types, clear is always constant time regardless of how many elements are in the container.
	s.col_clear();
	s.row_clear();
	
	//----------pivot A ----------//
	s.swap_clear();
	
	//------------- row-row swap (1) for A -------------//
	
	//pushes column indices (which contain non-zero elements) of A(k, 1:k) onto row_r
	for (idx_it it = list[k].begin(); it != list[k].begin() + row_first[k]; ++it) {
		s.row_r.push_back(*it);
	}
	
	//pushes column indices (which contain non-zero elements) of A(r, 1:k) onto row_k
	for (idx_it it = list[r].begin(); it != list[r].begin() + row_first[r]; ++it) {
		s.row_k.push_back(*it);
	}
	
	//merge these two sets of indices together
	s.all_swaps.assign(list[k].begin(), list[k].begin() + row_first[k]);
	unordered_inplace_union(s.all_swaps, list[r].begin(), list[r].begin() + row_first[r], in_set);
	
	//do row swaps in A (i.e. swap A(k, 1:k) with A(r, 1:k))
	for (idx_it it = s.all_swaps.begin(), end = s.all_swaps.end(); it != end; ++it) {
		safe_swap(m_idx[*it], k, r);
	}
	s.all_swaps.clear();
	
	//----------------------------------------------------//
	
	
	//---------------------- (2) and (3) for A --------------------------//
	
	//after sym. perm, a_rr will be swapped to a_kk, so we put a_rr as first
	//elem of col k if its non-zero. this also means that we ensure the first
	//elem of col k is the diagonal element if it exists.
	el_type elem = coeff(r, r);
	if (abs(elem) > eps){
		s.col_k_nnzs.push_back(k);
		s.col_k.push_back(elem);
	}
	
	//same as above, put a_kk in new col r if it exists.
	elem = coeff(k, k);
	if (abs(elem) > eps){
		s.col_r_nnzs.push_back(r);
		s.col_r.push_back(elem);
	}
	
	
	//first[r] should have # of nnz of A(r, 0:k-1)
	for (i = row_first[r]; i < (int) list[r].size(); i++) {
		j = list[r][i];
		assert(j >= k);
		if (coeffRef(r, j, its_k)) {
			if (j == k) {
				s.col_k_nnzs.push_back(r); //A(r, k) is fixed upon permutation so its index stays r
				s.row_r.push_back(k);
			} else {
				s.col_k_nnzs.push_back(j); //place A(r, j) (where k < j < r) into A(j, k)
			}
			s.col_k.push_back(*its_k.second);
			
			//delete A(r,j) from A.
			*its_k.first = m_idx[j].back();
			*its_k.second = m_x[j].back();
			
			m_idx[j].pop_back();
			m_x[j].pop_back();
		}
	}

	if (m_idx[r].size() > 0) {
	
		//place A(r:n, r) into A(r:n, k). since we already took care of A(r,r) above,
		//we need to offset by 1 if necessary.
        ensure_invariant(r, r, m_idx[r], false);
		offset = (m_idx[r][0] == r ? 1 : 0);
		std::copy(m_x[r].begin()+offset, m_x[r].end(), std::back_inserter(s.col_k));
		std::copy(m_idx[r].begin()+offset, m_idx[r].end(), std::back_inserter(s.col_k_nnzs));

		for (idx_it it = m_idx[r].begin() + offset; it != m_idx[r].end(); it++) {
		
			//for each non-zero row index in the rth column, find a pointer to it in list
			//these pointers will be used to perform column swaps on list
			for (i = row_first[*it]; i < (int) list[*it].size(); i++) {
				if (list[*it][i] == r) {
					s.swapk.push_back(list[*it].begin() + i);
					break;
				}
			}
		}
	}

	//swap A(k:r, k) with A(r, k:r)
	if (m_idx[k].size() > 0) {
		
		//since we already took care of A(k,k), we need an offset of 1 if necessary
        ensure_invariant(k, k, m_idx[k], false);
		offset = (m_idx[k][0] == k ? 1 : 0);
		for (i = offset; i < (int) m_idx[k].size(); i++) {
			idx = m_idx[k][i];
			
			//if idx < r, we are in (2) (row-col swap) otherwise we are in (3) (col-col swap)
			if (idx < r) {
			
				//swap A(i, k) with A(r, i) where k < i < r.
				m_idx[idx].push_back(r);	
				m_x[idx].push_back(m_x[k][i]);
				
				//we also have to ensure that list is updated by popping off old entries
				//that were meant for the A(i, k)'s before they were swapped.
				ensure_invariant(idx, k, list[idx], true);
				std::swap(list[idx][row_first[idx]], list[idx][list[idx].size() - 1]);
				list[idx].pop_back();
				
				//push back new elements on row_r
				s.row_r.push_back(idx);
				
			} else if (idx > r) {
			
				//swap A(i, k) with A(i, r) where r < i.
				s.col_r.push_back(m_x[k][i]);
				s.col_r_nnzs.push_back(idx);
				
				//for each non-zero row index in the kth column, find a pointer to it in list
				//these pointers will be used to perform column swaps on list
				for (j = row_first[idx]; j < (int) list[idx].size(); j++) {
					if (list[idx][j] == k) {
						s.swapr.push_back(list[idx].begin() + j);
						break;
					}
				}
			}
		}
	}

	//swap all A(i, k) with A(i, r) in list.
	for (typename vector<idx_it>::iterator it = s.swapk.begin(); it != s.swapk.end(); it++) {
		**it = k;
	}
	for (typename vector<idx_it>::iterator it = s.swapr.begin(); it != s.swapr.end(); it++) {
		**it = r;
	}

	//add new entries for new col k into list
	for (idx_it it = s.col_k_nnzs.begin(); it != s.col_k_nnzs.end(); it++) {
		if ((*it > k) && (*it < r)) {
			list[*it].push_back(k);
		}
	}
	
	//set the kth col
	m_idx[k].swap(s.col_k_nnzs);
	m_x[k].swap(s.col_k);
	
	//set the rth col
	m_idx[r].swap(s.col_r_nnzs);
	m_x[r].swap(s.col_r);
	
	//set the kth row and rth row
	list[k].swap(s.row_k);
	list[r].swap(s.row_r);

	//row swaps for first
	std::swap(row_first[k], row_first[r]);
	//--------end pivot A---------//
}
