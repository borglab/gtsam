/*!	\brief A structure containing variables used in pivoting a LIL-C matrix.
	
	Storing these variables in a combined structure reduces memory requirements and bundles together all temporary structures needed during pivoting.
*/
template<class el_type> 
class swap_struct
{
	//---------- useful typedefs (to keep consistent with lilc_matrix) -----------//
	typedef vector<int> idx_vector_type;
	typedef vector<el_type>  elt_vector_type;
	typedef typename idx_vector_type::iterator idx_it;
	typedef typename elt_vector_type::iterator elt_it;
	
	public:
		vector<idx_it> swapk;	///<List of indices from row r that will be swapped to row k.
		vector<idx_it> swapr;	///<List of indices from row k that will be swapped to row r.
		
		idx_vector_type all_swaps;	///<Column indices of all swaps done in swapk and swapr.
		idx_vector_type col_k_nnzs;	///<Row indices of non-zeros in the new column k.
		idx_vector_type col_r_nnzs;	///<Row indices of non-zeros in the new column r.
		
		elt_vector_type col_k;	///<Non-zero values in the new column k (order dependent on col_k_nnzs).
		elt_vector_type col_r;	///<Non-zero values in the new column r (order dependent on col_r_nnzs).
		
		idx_vector_type row_k;	///<Column indices of non-zeros in the new row k.
		idx_vector_type	row_r;	///<Column indices of non-zeros in the new row r.

		/*!	\brief Clears all swap vectors (swapk, swapr, all_swaps).
		*/
		void swap_clear() {
			swapk.clear();
			swapr.clear();
			all_swaps.clear();
		}
		
		/*!	\brief Clears all col vectors (col_k, col_r, col_k_nnzs, col_r_nnzs).
		*/
		void col_clear() {
			col_k.clear();
			col_k_nnzs.clear();
			col_r.clear();
			col_r_nnzs.clear();
		}
		
		/*!	\brief Clears all row vectors (row_k, row_r).
		*/
		void row_clear() {
			row_k.clear();
			row_r.clear();
		}
};