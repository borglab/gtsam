// -*- mode: c++ -*-
#ifndef _LILC_MATRIX_DECLARATIONS_H_
#define _LILC_MATRIX_DECLARATIONS_H_

#include <algorithm>
#include <cmath>
#include <cassert>
#include <iostream>
#include <iterator>
#include <set>

#include "swap_struct.h"

/*! \brief A list-of-lists (LIL) matrix in column oriented format.

	For convience, the matrix this class represents will be refered to as matrix A.
	In LIL-C format, each column of A (an n*n matrix) is stored as a separate vector. The nonzeros are stored in m_idx while the non-zeros are stored in m_x. Both m_x and m_idx are initialized to a list of n lists. m_idx and m_x are ordered dependent on each other, in that A(m_idx[k][j], k) = m_x[k][j].
	
*/

template<class el_type> 
class lilc_matrix : public lil_sparse_matrix<el_type>
{
public:
	
	//-------------- typedefs and inherited variables --------------//
	using lil_sparse_matrix<el_type>::m_idx;
	using lil_sparse_matrix<el_type>::m_x;
	using lil_sparse_matrix<el_type>::m_n_rows;
	using lil_sparse_matrix<el_type>::m_n_cols;
	using lil_sparse_matrix<el_type>::n_rows;
	using lil_sparse_matrix<el_type>::n_cols;
	using lil_sparse_matrix<el_type>::nnz;
	using lil_sparse_matrix<el_type>::nnz_count;
	using lil_sparse_matrix<el_type>::eps;


	typedef typename lil_sparse_matrix<el_type>::idx_vector_type idx_vector_type;
	typedef typename lil_sparse_matrix<el_type>::elt_vector_type elt_vector_type;
	
	typedef typename idx_vector_type::iterator idx_it;
	typedef typename elt_vector_type::iterator elt_it;

	std::vector< std::vector< int > > list;	///<A list of linked lists that gives the non-zero elements in each row of A. Since at any time we may swap between two rows, we require linked lists for each row of A.
	
	std::vector<int> row_first;	///<On iteration k, first[i] gives the number of non-zero elements on row i of A before A(i, k).
    std::vector<int> col_first;	///<On iteration k, first[i] gives the number of non-zero elements on col i of A before A(i, k).
    
	block_diag_matrix<el_type> S; ///<A diagonal scaling matrix S such that SAS will be equilibriated in the max-norm (i.e. every row/column has norm 1). S is constructed after running the sym_equil() function, after which SAS will be stored in place of A.
    
    //-------------- types of pivoting procedures ----------------//
    /*! A simple enum class for listing the type of pivoting procedure SYM-ILDL uses.
    */
	struct pivot_type {
		enum {
			BKP, 
			ROOK
		};
	};
	
public:
	
	/*! \brief Constructor for a column oriented list-of-lists (LIL) matrix. Space for both the values list and the indices list of the matrix is allocated here.
	*/
	lilc_matrix (int n_rows = 0, int n_cols = 0): 
	lil_sparse_matrix<el_type> (n_rows, n_cols) 
	{
		m_x.reserve(n_cols);
		m_idx.reserve(n_cols);
	}
	
	//----Matrix referencing/filling----//
	
	/*! \brief Finds the (i,j)th coefficient of the matrix.
		\param i the row of the (i,j)th element (zero-indexed).
		\param j the col of the (i,j)th element (zero-indexed).
		\param offset an optional search offset for use in linear search (start at offset instead of 0).
		\return The (i,j)th element of the matrix. 
	*/
	inline virtual el_type coeff(const int& i, const int& j, int offset = 0) const 
	{	
		//invariant: first elem in each col of a is the diagonal elem if it exists.
		if (i == j) {
			if (m_idx[j].size() == 0) return 0;
			return (m_idx[j][0] == i ? m_x[j][0] : 0);
		}
		
		for (int k = offset, end = m_idx[j].size(); k < end; k++) {
			if (m_idx[j][k] == i) return m_x[j][k];
		}
		
		return 0;
	}
	
	/*! \brief Finds the index/value pointers to (i,j)th coefficient of the matrix.
		\param i the row of the (i,j)th element (zero-indexed).
		\param j the col of the (i,j)th element (zero-indexed).
		\param its a pair of pointers, one for the index of the found element, and the other for the value of the element. If the element is not found, the pointers point to the end of column j.
		
		\return True if (i,j)th element is nonzero, false otherwise. 
	*/
	inline bool coeffRef(const int& i, const int& j, std::pair<idx_it, elt_it>& its)
	{	
		for (unsigned int k = 0; k < m_idx[j].size(); k++) {
			if (m_idx[j][k] == i) {
				its = make_pair(m_idx[j].begin() + k, m_x[j].begin() + k);
				return true;
			}
		}
		
		its = make_pair(m_idx[j].end(), m_x[j].end());
		return false;
	}
	
	/*! \brief Resizes the matrix. For use in preallocating space before factorization begins.
		\param n_rows the number of rows in the resized matrix.
		\param n_cols the number of cols in the resized matrix.
	*/
	void resize(int n_rows, int n_cols)
	{
		m_n_rows = n_rows;
		m_n_cols = n_cols;
	
		m_x.clear();
		m_idx.clear();
		row_first.clear();
		col_first.clear();
		list.clear();
	
		m_x.resize(n_cols);
		m_idx.resize(n_cols);
		
		row_first.resize(n_cols, 1);
		col_first.resize(n_cols, 1);
		list.resize(n_cols);
		
		S.resize(n_cols, 1);
		for (int i = 0; i < n_cols; i++) {
			m_x[i].clear();
			m_idx[i].clear();
			list[i].clear();
		}
	}
	
	//-----Reorderings/Rescalings------//
	/*!	\brief Returns a pseudo-peripheral root of A. This is essentially many chained breadth-first searchs across the graph of A (where A is viewed as an adjacency matrix).

		\param s contains the initial node to seed the algorithm. A pseudo-peripheral root of A is stored in s at the end of the algorithm.
	*/
	void find_root(int& s);
	
	/*!	\brief Returns the next level set given the current level set of A. This is essentially all neighbours of the currently enqueued nodes in breath-first search.
		
		\param lvl_set the current level set (a list of nodes).
		\param visited all previously visited nodes.
	*/
	inline bool find_level_set(vector<int>& lvl_set, vector<bool>& visited);
	
	/*!	\brief Returns a Reverse Cuthill-McKee ordering of the matrix A (stored in perm). 
		
		A detailed description of this function as well as all its subfunctions can be found in "Computer Solution of Large Sparse Positive Definite Systems" by George and Liu (1981).
		\param perm An empty permutation vector (filled on function completion).
	*/
	void sym_rcm(vector<int>& perm);
	
	/*! \brief Returns a Approximate Minimum Degree ordering of the matrix A (stored in perm). 
		
		A detailed description of this function as well as all its subfunctions can be found in "An Approximate Minimum Dgree Algorithm" by Davis, Amestoy, and Duff (1981).
		\param perm An empty permutation vector (filled on function completion).
	*/
	inline void sym_amd(vector<int>& perm);
		
	/*! \brief Given a permutation vector perm, A is permuted to P'AP, where P is the permutation matrix associated with perm. 
		\param perm the permutation vector.
	*/
	void sym_perm(vector<int>& perm);
	
	/*!	\brief The symmetric matrix A is equilibrated and the symmetric equilibrated matrix SAS is stored in A, where S is a diagonal scaling matrix. 
		
		This algorithm is based on the one outlined in "Equilibration of Symmetric Matrices in the Max-Norm" by Bunch (1971).
	*/
	void sym_equil();
	
	//----Factorizations----//
	/*! \brief Performs an LDL' factorization of this matrix. 
		
		The pivoted matrix P'AP will be stored in place of A. In addition, the L and D factors of P'AP will be stored in L and D (so that P'AP = LDL'). The factorization is performed in crout order and follows the algorithm outlined in "Crout versions of the ILU factorization with pivoting for sparse symmetric matrices" by Li and Saad (2005).
	
		\param L the L factor of this matrix.
		\param D the D factor of this matrix.
		\param perm the current permutation of A.
		\param fill_factor a parameter to control memory usage. Each column is guaranteed to have fewer than fill_factor*(nnz(A)/n_col(A)) elements.
		\param tol a parameter to control agressiveness of dropping. In each column, elements less than tol*norm(column) are dropped.
	    \param pp_tol a parameter to control aggresiveness of pivoting. Allowable ranges are [0,inf). If the parameter is >= 1, Bunch-Kaufman pivoting will be done in full. If the parameter is 0, partial pivoting will be turned off and the first non-zero pivot under the diagonal will be used. Choices close to 0 increase locality in pivoting (pivots closer to the diagonal are used) while choices closer to 1 increase the stability of pivoting. Useful for situations where you care more about preserving the structure of the matrix rather than bounding the size of its elements.
        \param pivot_type chooses the type of pivoting procedure used: threshold Bunch-Kaufman, or rook pivoting. If rook pivoting is chosen, pp_tol is ignored.
	*/
	void ildl(lilc_matrix<el_type>& L, block_diag_matrix<el_type>& D, idx_vector_type& perm, const double& fill_factor, const double& tol, const double& pp_tol, int piv_type = pivot_type::BKP);
	
    /*! \brief Performs an _inplace_ LDL' factorization of this matrix. 
		
		The pivoted matrix P'AP will be stored in place of A. In addition, the L and D factors of P'AP will be stored in L and D (so that P'AP = LDL'). The factorization is performed in crout order and follows the algorithm outlined in "Crout versions of the ILU factorization with pivoting for sparse symmetric matrices" by Li and Saad (2005).
	
		\param D the D factor of this matrix.
		\param perm the current permutation of A.
		\param fill_factor a parameter to control memory usage. Each column is guaranteed to have fewer than fill_factor*(nnz(A)/n_col(A)) elements.
		\param tol a parameter to control agressiveness of dropping. In each column, elements less than tol*norm(column) are dropped.
	    \param pp_tol a parameter to control aggresiveness of pivoting. Allowable ranges are [0,inf). If the parameter is >= 1, Bunch-Kaufman pivoting will be done in full. If the parameter is 0, partial pivoting will be turned off and the first non-zero pivot under the diagonal will be used. Choices close to 0 increase locality in pivoting (pivots closer to the diagonal are used) while choices closer to 1 increase the stability of pivoting. Useful for situations where you care more about preserving the structure of the matrix rather than bounding the size of its elements.
	*/
	void ildl_inplace(block_diag_matrix<el_type>& D, idx_vector_type& perm, const double& fill_factor, const double& tol, const double& pp_tol, int piv_type = pivot_type::BKP);
    
	//------Helpers------//
	/*! \brief Performs a back solve of this matrix, assuming that it is lower triangular (stored column major). 
		
		\param b the right hand side.
		\param x a storage vector for the solution (must be same size as b).
	*/
	void backsolve(const elt_vector_type& b, elt_vector_type& x) {
		assert(b.size() == x.size());
		x = b;
		// simple forward substitution
		for (int i = 0; i < m_n_cols; i++) {
			x[i] /= m_x[i][0];
			for (int k = 1; k < m_idx[i].size(); k++) {
				x[m_idx[i][k]] -= x[i]*m_x[i][k];
			}
		}
	}
	
	/*! \brief Performs a forward solve of this matrix, assuming that it is upper triangular (stored row major).
		
		\param b the right hand side.
		\param x a storage vector for the solution (must be same size as b).
	*/
	void forwardsolve(const elt_vector_type& b, elt_vector_type& x) {
		assert(b.size() == x.size());
		// simple back substitution
		for (int i = m_n_cols-1; i >= 0; i--) {
			x[i] = b[i]/m_x[i][0];
			for (int k = 1; k < m_idx[i].size(); k++) {
				x[i] -= x[m_idx[i][k]]*m_x[i][k]/m_x[i][0];
			}
		}
	}
	
	/*! \brief Performs a matrix-vector product with this matrix.
		
		\param x the vector to be multiplied.
		\param y a storage vector for the result (must be same size as x).
		\param full_mult if true, we assume that only half the matrix is stored and do do operations per element of the matrix to account for the unstored other half.
	*/
	void multiply(const elt_vector_type& x, elt_vector_type& y, bool full_mult = true) {
		y.clear(); y.resize(x.size(), 0);
		for (int i = 0; i < m_n_cols; i++) {
			for (int k = 0; k < m_idx[i].size(); k++) {
				y[m_idx[i][k]] += x[i]*m_x[i][k];
				if (full_mult && i != m_idx[i][k]) {
					y[i] += x[m_idx[i][k]]*m_x[i][k];
				}
			}
		}
	}
	
	/*! \brief Performs a symmetric permutation between row/col k & r of A.
	
		\param s a struct containing temporary variables needed during pivoting.
		\param in_set a bitset needed for unordered unions during pivoting.
		\param L the lower triangular factor of A.
		\param k index of row/col k.
		\param r index of row/col r.
	*/
	inline void pivot(swap_struct<el_type>& s, vector<bool>& in_set, lilc_matrix<el_type>& L, const int& k, const int& r);
    
    /*! \brief The inplace version of the function above.
    
        \param s a struct containing temporary variables needed during pivoting.
		\param in_set a bitset needed for unordered unions during pivoting.
		\param k index of row/col k.
		\param r index of row/col r.
    */
	inline void pivotA(swap_struct<el_type>& s, vector<bool>& in_set, const int& k, const int& r);
	
	/*! \brief Ensures two the invariants observed by A.first and A.list are held.
		
		\invariant
		If this matrix is a lower triangular factor of another matrix:
			-# On iteration k, first[i] will give the number of non-zero elements on col i of A before A(k, i).
			-# On iteration k, list[i][ first[i] ] will contain the first element below or on index k of column i of A.
		
		\invariant
		If this matrix is the matrix to be factored:
			-# On iteration k, first[i] will give the number of non-zero elements on row i of A before A(i, k).
			-# On iteration k, list[i][ first[i] ] will contain the first element right of or on index k of row i of A.
			
		\param j the column of con.
		\param k the iteration number.
		\param con the container to be swapped.
		\param update_list boolean indicating whether list or m_x/m_idx should be updated.
	*/
	template <class Container>
	inline void ensure_invariant(const int& j, const int& k, Container& con, bool update_list = false) {
		int offset;
        if (update_list) offset = row_first[j];
        else offset = col_first[j];
        
		if ((offset >= (int) con.size()) || con.empty() || con[offset] == k) return;
		
		int i, min(offset);
		for (i = offset; i < (int) con.size(); i++) {
			if (con[i] == k) {
				min = i; 
				break;
			} else if ( con[i] < con[min] ) {
				min = i;
			}
		}
		
		if (update_list)
			std::swap(con[offset], con[min]);
		else {
			std::swap(con[offset], con[min]);
			std::swap(m_x[j][offset], m_x[j][min]);
		}
	}
	
	/*! \brief Updates A.first for iteration k.
		\param k current iteration index.	
	*/
	inline void advance_first(const int& k) {
		for (idx_it it = list[k].begin(); it != list[k].end(); it++) {	
			ensure_invariant(*it, k, m_idx[*it]); //make sure next element is good before we increment.
			col_first[*it]++; //should have ensured invariant now
		}
	}
	
	/*! \brief Updates A.list for iteration k.
		\param k current iteration index.	
	*/
	inline void advance_list(const int& k) {
		for (idx_it it = m_idx[k].begin(); it != m_idx[k].end(); it++) {
            if (*it == k) continue;
			ensure_invariant(*it, k, list[*it], true); //make sure next element is good.
            row_first[*it]++; //invariant ensured.
		}			
	}
	
	//----IO Functions----//
	
	/*! \brief Returns a string representation of A, with each column and its corresponding indices & non-zero values printed.
		\return A string representation of this matrix.
	*/

	std::string to_string () const;
	
	/*! \brief Loads a matrix in matrix market format.
		\param filename the filename of the matrix to be loaded. Must be in matrix market format (.mtx).
	*/
	bool load(std::string filename);

	/*! \brief Loads a matrix in CSC format.
		\param ptr A vector containing the ranges of indices in each col.
		\param row A vector containing the row indices of the nnz.
		\param val A vector containing the values of the non-zeros.
	*/
	bool load(const std::vector<int>& ptr, const std::vector<int>& row, const std::vector<el_type>& val);

	/*! \brief Loads a matrix in CSC format. Does no error checking on the input vectors.
		\param row A vector containing the row indices of the nnz.
		\param ptr A vector containing the ranges of indices in each col.
		\param val A vector containing the values of the non-zeros.
		\param dim The dimension of the matrix.
	*/	
	bool load(const int* ptr, const int* row, const el_type* val, int dim);
	
	/*! \brief Saves a matrix in matrix market format.
		\param filename the filename of the matrix to be saved. All matrices saved are in matrix market format (.mtx).
		\param sym flags whether the matrix is symmetric or not.
	*/
	bool save(std::string filename, bool sym = false);

};

//------------------ include files for class functions -------------------//

#include "lilc_matrix_find_level_set.h"
#include "lilc_matrix_find_root.h"
#include "lilc_matrix_sym_rcm.h"
#include "lilc_matrix_sym_amd.h"
#include "lilc_matrix_sym_perm.h"
#include "lilc_matrix_sym_equil.h"
#include "lilc_matrix_ildl_helpers.h"
#include "lilc_matrix_ildl.h"
#include "lilc_matrix_ildl_inplace.h"
#include "lilc_matrix_pivot.h"
#include "lilc_matrix_load.h"
#include "lilc_matrix_save.h"
#include "lilc_matrix_to_string.h"

#endif
