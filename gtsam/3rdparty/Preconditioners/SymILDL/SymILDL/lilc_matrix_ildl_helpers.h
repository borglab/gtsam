#ifndef _LILC_MATRIX_ILDL_HELPERS_H
#define _LILC_MATRIX_ILDL_HELPERS_H

using std::abs;
using std::vector;

typedef vector<int>::iterator idx_it;

/*! \brief Computes the dot product of v and w. Only works when el_type is real right now.
	\param v the first vector whose dot product we wish to compute
	\param w the second vector whose dot product we wish to compute
*/
template <class el_type>
inline double dot_product(const vector<el_type>& v, const vector<el_type>& w) {
	double res = 0;
	for (int i = 0; i < v.size(); i++) {
		res += v[i]*w[i];
	}
	return res;
}

/*! \brief Computes the sum of two vectors: u = a*v + b*w. a and b are scalars, u, v, and w are vectors.
	\param u the storage vector for the result
*/
template <class el_type>
inline void vector_sum(double a, vector<el_type>& v, double b, vector<el_type>& w, vector<el_type>& u) {
	for (int i = 0; i < v.size(); i++) {
		u[i] = a*v[i] + b*w[i];
	}
}

/*! \brief Computes the maximum (in absolute value) element of v(curr_nnzs) and it's index.
	\param v the vector whose max element is to be computed.
	\param curr_nnzs a list of indices representing non-zero elements in v.
	\param r the index of the maximum element of v
	
	\return the max element of v.
*/
template <class el_type>
inline double max(vector<el_type>& v, vector<int>& curr_nnzs, int& r) { 
	double res = 0;
	for (idx_it it = curr_nnzs.begin(), end = curr_nnzs.end(); it != end; ++it) {
		if (abs(v[*it]) > res) {
			res = abs(v[*it]);
			r = *it;
		}
	}
	
	return res;
}

/*! \brief Computes the norm of v(curr_nnzs).
	\param v the vector whose norm is to be computed.
	\param curr_nnzs a list of indices representing non-zero elements in v.
	\param p The norm number.
	\return the norm of v.
*/
template <class el_type>
inline double norm(vector<el_type>& v, vector<int>& curr_nnzs, el_type p = 1) { 
	el_type res = 0;
	for (idx_it it = curr_nnzs.begin(), end = curr_nnzs.end(); it != end; ++it) {
		res += pow(abs(v[*it]), p);  
	}
	
	return pow(res, 1/p);
}

/*! \brief Computes the norm of v.
	\param v the vector whose norm is to be computed.
	\param p The norm number.
	\return the norm of v.
*/
template <class el_type>
inline double norm(vector<el_type>& v, el_type p = 1) { 
	el_type res = 0;
	for (int i = 0; i < v.size(); i++) {
		res += pow(abs(v[i]), p);
	}
	return pow(res, 1/p);
}

/*! \brief Performs an inplace union of two sorted lists (a and b), removing duplicates in the final list.
	\param a the sorted list to contain the final merged list.
	\param b_start an iterator to the start of b.
	\param b_end an iterator to the end of b.
*/
template <class InputContainer, class InputIterator>
inline void inplace_union(InputContainer& a, InputIterator const& b_start, InputIterator const& b_end)
{
	int mid = a.size(); //store the end of first sorted range

	//copy the second sorted range into the destination vector
	std::copy(b_start, b_end, std::back_inserter(a));

	//perform the in place merge on the two sub-sorted ranges.
	std::inplace_merge(a.begin(), a.begin() + mid, a.end());

	//remove duplicate elements from the sorted vector
	a.erase(std::unique(a.begin(), a.end()), a.end());
}

/*! \brief Performs an inplace union of two unsorted lists (a and b), removing duplicates in the final list.
	\param a the sorted list to contain the final merged list.
	\param b_start an iterator to the start of b.
	\param b_end an iterator to the end of b.
	\param in_set a bitset used to indicate elements present in a and b. Reset to all zeros after union.
*/
template <class InputContainer, class InputIterator>
inline void unordered_inplace_union(InputContainer& a, InputIterator const& b_start, InputIterator const& b_end, vector<bool>& in_set)
{
	for (InputIterator it = a.begin(), end = a.end(); it != end; ++it) {
        assert(*it < in_set.size() && *it >= 0);
		in_set[*it] = true;
	}
	
	for (InputIterator it = b_start; it != b_end; ++it) {
		if (!in_set[*it]) {
			in_set[*it] = true;
			a.push_back(*it);
		}
	}
	
	for (InputIterator it = a.begin(), end = a.end(); it != end; ++it) {
        assert(*it < in_set.size() && *it >= 0);
		in_set[*it] = false;
	}
}

//-------------Dropping rules-------------//


namespace {
/*! \brief Functor for comparing elements by value (in decreasing order) instead of by index.
	\param v the vector that contains the values being compared.
*/
template <class el_type>
struct by_value {
	const vector<el_type>& v; 
	by_value(const vector<el_type>& vec) : v(vec) {}
	inline bool operator()(int const &a, int const &b) const { 
		// Not needed if we're using sort. If using this comparator
		// in a set, then uncomment the line below.
		//if (abs(v[a]) == abs(v[b])) return a < b;
		return abs(v[a]) > abs(v[b]);
	}
};

/*! \brief Functor for determining if a variable is below the tolerance given.
    \param v the vector that contains the values being checked.
    \param eps the tolerance given.
*/
template <class el_type>
struct by_tolerance {
  const vector<el_type>& v; 
  double eps;
	by_tolerance(const vector<el_type>& vec, const double& eps) : v(vec), eps(eps) {}
	inline bool operator()(int const &i) const { 
		return abs(v[i]) < eps;
	}
};
}

/*! \brief Performs the dual-dropping criteria outlined in Li & Saad (2005).
	\param v the vector that whose elements will be selectively dropped.
	\param curr_nnzs the non-zeros in the vector v.
	\param lfil a parameter to control memory usage. Each column is guarannted to have fewer than lfil elements.
	\param tol a parameter to control agressiveness of dropping. Elements less than tol*norm(v) are dropped.
*/
template <class el_type>
inline void drop_tol(vector<el_type>& v, vector<int>& curr_nnzs, const int& lfil, const double& tol) { 
	//determine dropping tolerance. all elements with value less than tolerance = tol * norm(v) is dropped.
	el_type tolerance = tol*norm<el_type>(v, curr_nnzs);
	const long double eps = 1e-13; //TODO: fix later. need to make this a global thing
	if (tolerance > eps) {
		for (idx_it it = curr_nnzs.begin(), end = curr_nnzs.end(); it != end; ++it) 
		if (abs(v[*it]) < tolerance) v[*it] = 0;
		
		//sort the remaining elements by value in decreasing order.
		by_value<el_type> sorter(v);
		std::sort(curr_nnzs.begin(), curr_nnzs.end(), sorter);
	}
	
	for (int i = lfil, end = curr_nnzs.size(); i < end ; ++i) {
		v[curr_nnzs[i]] = 0;
	}
	
    by_tolerance<el_type> is_zero(v, eps);
	curr_nnzs.erase( remove_if(curr_nnzs.begin(), curr_nnzs.end(), is_zero), curr_nnzs.end() );
	curr_nnzs.resize( std::min(lfil, (int) curr_nnzs.size()) );
	//sort the first lfil elements by index, only these will be assigned into L. this part can be removed.
	std::sort(curr_nnzs.begin(), curr_nnzs.end());
}

//----------------Column updates------------------//

template <class el_type>
inline void update_single(const int& k, const int& j, const el_type& l_ki, const el_type& d, vector<el_type>& work, vector<int>& curr_nnzs, lilc_matrix<el_type>& L, vector<bool>& in_set) {
	//find where L(k, k+1:n) starts
	int i, offset = L.col_first[j];
    
	L.ensure_invariant(j, k, L.m_idx[j]);
    
	el_type factor = l_ki * d;
	for (i = offset; i < L.m_idx[j].size(); ++i) {
		int x = L.m_idx[j][i];
		work[x] -= factor * L.m_x[j][i];
		if (!in_set[x]) {
			curr_nnzs.push_back(x);
			in_set[x] = true;
		}
	}
}

/*! \brief Performs a delayed update of subcolumn A(k:n,r). Result is stored in work vector. Nonzero elements of the work vector are stored in curr_nnzs.
	\param r the column number to be updated.
	\param work the vector for which all delayed-updates are computed to.
	\param curr_nnzs the nonzero elements of work.
	\param L the (partial) lower triangular factor of A.
	\param D the (partial) diagonal factor of A.
	\param in_set temporary storage for use in merging two lists of nonzero indices.
*/
template <class el_type>
inline void update(const int& r, vector<el_type>& work, vector<int>& curr_nnzs, lilc_matrix<el_type>& L, block_diag_matrix<el_type>& D, vector<bool>& in_set) {
	unsigned int j;
	int blk_sz;
	el_type d_12, l_ri;	

	// precord: in_set is all false.
	// TOOD: if we are for sure doing sequential code, make in_set a static var.
	for (int x : curr_nnzs) {
		in_set[x] = true;
	}

	//iterate across non-zeros of row k using Llist
	for (int i = 0; i < (int) L.list[r].size(); ++i) {
		j = L.list[r][i];        
        assert(j < r);
		l_ri = L.coeff(r, j, L.col_first[j]);
        
		update_single(r, j, l_ri, D[j], work, curr_nnzs, L, in_set); //update col using d11
		
		blk_sz = D.block_size(j);
		if (blk_sz == 2) {
			d_12 = D.off_diagonal(j);
			update_single(r, j + 1, l_ri, d_12, work, curr_nnzs, L, in_set);
		} else if (blk_sz == -2) {
			d_12 = D.off_diagonal(j-1);
			update_single(r, j - 1, l_ri, d_12, work, curr_nnzs, L, in_set); //update col using d12
		}
		
	}

	for (int x : curr_nnzs) {
		in_set[x] = false;
	}
}

//not needed anymore
template <class el_type>
inline void vec_add(vector<el_type>& v1, vector<int>& v1_nnzs, vector<el_type>& v2, vector<int>& v2_nnzs) {
	//merge current non-zeros of col k with nonzeros of col *it. 
	inplace_union(v1_nnzs, v2_nnzs.begin(), v2_nnzs.end());
	for (idx_it it = v1_nnzs.begin(), end = v1_nnzs.end(); it != end; ++it) {
		v1[*it] += v2[*it];
	}
}

inline void safe_swap(vector<int>& curr_nnzs, const int& k, const int& r) {
	bool con_k = false, con_r = false;
	vector<int>::iterator k_idx, r_idx;
	for (idx_it it = curr_nnzs.begin(), end = curr_nnzs.end(); it != end; ++it) {
		if (*it == k) {
			con_k = true;
			k_idx = it;
		}
		
		if (*it == r) {
			con_r = true;
			r_idx = it;
		}
	}
	
	if (con_k) *k_idx = r;  //if we have k we'll swap index to r
	if (con_r) *r_idx = k;  //if we have r we'll swap index to k
}

#endif
