#ifndef _SOLVER_H
#define _SOLVER_H

#include <iostream>
#include <cstring>
#include <ctime>
#include <iomanip>

#include "lilc_matrix.h"

namespace symildl {

// Using struct'd enums to achieve a C++11 style enum class without C++11
struct reordering_type {
	enum {
		NONE,
		AMD,
		RCM,
		MC64
	};
};

struct equilibration_type {
	enum {
		NONE,
		BUNCH,
		RUIZ,
		MC64
	};
};

struct solver_type {
	enum {
		NONE,
		MINRES,
		SQMR,
		FULL
	};
};

struct message_level {
	enum {
		NONE,
		STATISTICS,
		DEBUG
	};
};


/*!	\brief Saves a permutation vector vec as a permutation matrix in matrix market (.mtx) format.
	\param vec the permutation vector.
	\param filename the filename the matrix will be saved under.
*/
template<class el_type>
bool save_vector(const std::vector<el_type>& vec, std::string filename) {
	std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary);
	if(!out)
	return false;

	out.flags(std::ios_base::scientific);
	out.precision(12);
	std::string header = "%%MatrixMarket matrix coordinate real general";; 

	out << header << std::endl; 
	out << vec.size() << " " << 1 << " " << vec.size() << "\n";

	for(int i = 0; i < (int) vec.size(); i++) {
		out << i+1 << " " << 1 << " " << vec[i] << "\n";
	}
	
	out.close();
	return true;
}

/*!	\brief Reads in a dense row or column vector vec in matrix market (.mtx) format.
	\param rhs the permutation vector.
	\param filename the filename the matrix will be saved under.
*/
template<class el_type>
bool read_vector(std::vector<el_type>& vec, std::string filename, int msg_lvl = message_level::STATISTICS) {
	std::ifstream input(filename.c_str(), std::ios::in);

	if(!input) return false;
	
	const int maxBuffersize = 2048;
	char buffer[maxBuffersize];

	bool readsizes = false;
	el_type value; 

	int i = 0, n_rows, n_cols;
	while(input.getline(buffer, maxBuffersize)) {
		// skip comments   
		//NOTE An appropriate test should be done on the header to get the symmetry
		if(buffer[0]=='%') continue;
		
		std::stringstream line(buffer);
		
		if(!readsizes) {
			line >> n_rows >> n_cols;
			if(n_rows > 0 && n_cols > 0) {
				readsizes = true;
				vec.resize(std::max(n_rows, n_cols));
			}
		} else {
			line >> value;
			vec[i++] = value;
		}
	}
	
	if (i != std::max(n_rows, n_cols)) {
		std::cerr << "Expected " << std::max(n_rows, n_cols) << " elems but read " << i << "." << std::endl;
	}
	
	if (msg_lvl) std::cout << "Load succeeded. " << "Vector file " << filename << " was loaded." << std::endl;
	input.close();
	return true;
}

/*! \brief Set of tools that facilitates conversion between different matrix formats. Also contains solver methods for matrices using a common interface.

	Currently, the only matrix type accepted is the lilc_matrix (as no other matrix type has been created yet).
*/
template<class el_type, class mat_type = lilc_matrix<el_type> >
class solver {
	public:
        typedef typename mat_type::pivot_type pivot_type;
        
		mat_type A;	///<The matrix to be factored.
		mat_type L;	///<The lower triangular factor of A.
		
        vector<int> perm;	///<A permutation vector containing all permutations on A.
		block_diag_matrix<el_type> D;	///<The diagonal factor of A.
		int reorder_type; ///<Set to to 0 for AMD, 1 for RCM, 2 for no reordering.
        int piv_type; ///<Set to 0 for rook, 1 for bunch.
		
        int equil_type; ///<The equilibration method used. Set to 1 for max-norm equilibriation.
		
	int msg_lvl; ///<Controls the amount of output to stdout.
		int solve_type; //<The type of solver used to solve the right hand side.
		bool has_rhs; ///<Set to true if we have a right hand side that we expect to solve.
		bool perform_inplace; ///<Set to true if we are factoring the matrix A inplace.
       		// TODO: refactor this away
		bool save_sol; ///<Set to true if we want to save the solution to a file.
		 
        vector<el_type> rhs; ///<The right hand side we'll solve for.
		vector<el_type> sol_vec; ///<The solution vector.
		
		/*! \brief Solver constructor, initializes default reordering scheme.
		*/
		solver() {
			msg_lvl = message_level::STATISTICS;
            		piv_type = pivot_type::ROOK;
			reorder_type = reordering_type::AMD;
			equil_type = equilibration_type::BUNCH;
			solve_type = solver_type::SQMR;
            		has_rhs = false;
            		perform_inplace = false;
		}
				
		/*! \brief Loads the matrix A into solver. A must be of matrix market format.
			\param filename the filename of the matrix.
		*/
		void load(std::string filename) {
			bool result = A.load(filename);
			assert(result);
			if (msg_lvl) printf("A is %d by %d with %d non-zeros.\n", A.n_rows(), A.n_cols(), A.nnz() );
		}

		/*! \brief Loads the matrix A into solver. A must be of CSC format.
		*/
		void load(const std::vector<int>& ptr, const std::vector<int>& row, const std::vector<el_type>& val) {
			bool result = A.load(ptr, row, val);
			assert(result);	
			if (msg_lvl) printf("A is %d by %d with %d non-zeros.\n", A.n_rows(), A.n_cols(), A.nnz() );
		}

		/*! \brief Loads the matrix A into solver. A must be of CSC format.
		*/
		void load(const int* ptr, const int* row, const el_type* val, int dim) {
			bool result = A.load(ptr, row, val, dim);
			assert(result);	
			if (msg_lvl) printf("A is %d by %d with %d non-zeros.\n", A.n_rows(), A.n_cols(), A.nnz() );
		}

		
		/*! \brief Loads a right hand side b into the solver.
			\param b a vector of the right hand side.
		*/
		void set_rhs(vector<el_type> b) {
			rhs = b;
			has_rhs = true;
			if (msg_lvl) printf("Right hand side has %d entries.\n", rhs.size() );
		}
		
		/*! \brief Sets the reordering scheme for the solver.
		*/
		void set_reorder_scheme(const char* ordering) {
			if (strcmp(ordering, "rcm") == 0) {
				reorder_type = reordering_type::RCM;
			} else if (strcmp(ordering, "amd") == 0) {
				reorder_type = reordering_type::AMD;
			} else if (strcmp(ordering, "none") == 0) {
				reorder_type = reordering_type::NONE;
			}
		}
		
		/*! \brief Decides whether we should use equilibriation on the matrix or not.
		*/
		void set_equil(const char* equil) {
			if (strcmp(equil, "bunch") == 0) {
				equil_type = equilibration_type::BUNCH;
			} else if (strcmp(equil, "none") == 0) {
				equil_type = equilibration_type::NONE;
			}
		}
        
		/*! \brief Decides whether we perform a full solve or not.
		*/
		void set_solver(const char* solver) {
			if (strcmp(solver, "minres") == 0) {
				solve_type = solver_type::MINRES;
			} else if (strcmp(solver, "sqmr") == 0) {
				solve_type = solver_type::SQMR;
			} else if (strcmp(solver, "full") == 0) {
				solve_type = solver_type::FULL;
			} else if (strcmp(solver, "none") == 0) {
				solve_type = solver_type::NONE;
			}
		}

		/*! \brief Controls how much information gets printed to stdout.
		*/
		void set_message_level(const char* msg) {
			if (strcmp(msg, "none") == 0) {
				msg_lvl = message_level::NONE;
			} else if (strcmp(msg, "statistics") == 0) {
				msg_lvl = message_level::STATISTICS;
			} else if (strcmp(msg, "debug") == 0) {
				msg_lvl = message_level::DEBUG;
			}
		}
		
        /*! \brief Decides whether we perform the factorization inplace or not.
		*/
		void set_inplace(bool inplace) {
			perform_inplace = inplace;
		}
        
        /*! \brief Decides the kind of partial pivoting we should use.
		*/
		void set_pivot(const char* pivot) {
			if (strcmp(pivot, "rook") == 0) {
                piv_type = pivot_type::ROOK;
            } else if (strcmp(pivot, "bunch") == 0) {
                piv_type = pivot_type::BKP;
            }
		}

		/*! \brief Factors the matrix A into P' * S * A * S * P = LDL' in addition to printing some timing data to screen.
			
			More information about the parameters can be found in the documentation for the ildl() function.
			
			\param fill_factor a factor controling memory usage of factorization.
			\param tol a factor controling accuracy of factorization.
			\param pp_tol a factor controling the aggresiveness of Bunch-Kaufman pivoting.
			\param max_iter the maximum number of iterations for minres (ignored if no right hand side).
		*/
		void solve(double fill_factor, double tol, double pp_tol, int max_iter = -1, double minres_tol = 1e-6, double shift = 0.0) {
            // A full factorization is equivalent to a fill factor of n and tol of 0
            if (solve_type == solver_type::FULL) {
                tol = 0.0;
                fill_factor = A.n_cols();
            }
            
			perm.reserve(A.n_cols());
			cout << std::fixed << std::setprecision(3);
			
			double dif, total = 0;
			clock_t start;
			
			if (equil_type == equilibration_type::BUNCH) {
				start = clock();
				A.sym_equil();
				dif = clock() - start; total += dif; 
				if (msg_lvl) printf("  Equilibration:\t\t%.3f seconds.\n", dif/CLOCKS_PER_SEC);
			}

			if (reorder_type != reordering_type::NONE) {
				start = clock();
				std::string perm_name;
				switch (reorder_type) {
					case reordering_type::AMD:
						A.sym_amd(perm);
						perm_name = "AMD";
						break;
					case reordering_type::RCM:
						A.sym_rcm(perm);
						perm_name = "RCM";
						break;
				}
				
				dif = clock() - start; total += dif;
				if (msg_lvl) printf("  %s:\t\t\t\t%.3f seconds.\n", perm_name.c_str(), dif/CLOCKS_PER_SEC);
				
				start = clock();
				A.sym_perm(perm);
				dif = clock() - start; total += dif;
				if (msg_lvl) printf("  Permutation:\t\t\t%.3f seconds.\n", dif/CLOCKS_PER_SEC);
			} else {
				// no permutation specified, store identity permutation instead.
				for (int i = 0; i < A.n_cols(); i++) {
					perm.push_back(i);
				}
			}

			start = clock();
            if (perform_inplace) {
                A.ildl_inplace(D, perm, fill_factor, tol, pp_tol, piv_type);
            } else {
                A.ildl(L, D, perm, fill_factor, tol, pp_tol, piv_type);
            }
			dif = clock() - start; total += dif;
			
            std::string pivot_name;
            if (piv_type == pivot_type::BKP) {
                pivot_name = "BK";
            } else if (piv_type == pivot_type::ROOK) {
                pivot_name = "Rook";
            }
            
			if (msg_lvl) printf("  Factorization (%s pivoting):\t%.3f seconds.\n", pivot_name.c_str(), dif/CLOCKS_PER_SEC);
			if (msg_lvl) printf("Total time:\t\t\t%.3f seconds.\n", total/CLOCKS_PER_SEC);
            if (perform_inplace) {
                if (msg_lvl) printf("L is %d by %d with %d non-zeros.\n", A.n_rows(), A.n_cols(), A.nnz() );
            } else {
                if (msg_lvl) printf("L is %d by %d with %d non-zeros.\n", L.n_rows(), L.n_cols(), L.nnz() );
            }
			if (msg_lvl) printf("\n");
			fflush(stdout);
			
			// if there is a right hand side, it means the user wants a solve.
			// TODO: refactor this solve to be in its own method, and separate 
			// factoring/minres solve phase
			if (has_rhs) {
                if (perform_inplace) {
                    if (msg_lvl) printf("Inplace factorization cannot be used with the solver. Please try again without -inplace.\n");
                } else {
                    // start timer in case we're doing a full solve
                    start = clock();
                    
                    // we've permuted and equilibrated the matrix, so we gotta apply 
                    // the same permutation and equilibration to the right hand side.
                    // i.e. rhs = P'S*rhs
                    // 0. apply S
                    for (int i = 0; i < A.n_cols(); i++) {
                        rhs[i] = A.S[i]*rhs[i];
                    }
                    
                    // 1. apply P' (takes rhs[perm[i]] to rhs[i], i.e. inverse of perm, 
                    //    where perm takes i to perm[i])
                    vector<el_type> tmp(A.n_cols());
                    for (int i = 0; i < A.n_cols(); i++) {
                        tmp[i] = rhs[perm[i]];
                    }
                    rhs = tmp;
                    
                    if (solve_type == solver_type::FULL) {
                        if (msg_lvl) printf("Solving matrix with direct solver...\n");
                        sol_vec.resize(A.n_cols(), 0);
                        // MINRES uses the preconditioned solver that
                        // splits the block D into |D|^(1/2).
                        // For the full solver we'll just solve D directly.
                        L.backsolve(rhs, sol_vec);
                        D.solve(sol_vec, tmp);
                        L.forwardsolve(tmp, sol_vec);
                    } else {
                        start = clock();
						
						if (solve_type == solver_type::MINRES) {
							// finally, since we're preconditioning with M = L|D|^(1/2), we have
							// to multiply M^(-1) to the rhs and solve the system
							// M^(-1) * B * M'^(-1) y = M^(-1)P'*S*b
							L.backsolve(rhs, tmp);
							D.sqrt_solve(tmp, rhs, false);
							
							if (msg_lvl) printf("Solving matrix with MINRES...\n");
							// solve the equilibrated, preconditioned, and permuted linear system
							minres(max_iter, minres_tol, shift);
							
							// now we've solved M^(-1)*B*M'^(-1)y = M^(-1)P'*S*b
							// where B = P'SASPy.
							
							// but the actual solution is y = M' * P'S^(-1)*x
							// so x = S*P*M'^(-1)*y
							
							// 0. apply M'^(-1)
							D.sqrt_solve(sol_vec, tmp, true);
							L.forwardsolve(tmp, sol_vec);
						} else if (solve_type == solver_type::SQMR) {
							if (msg_lvl) printf("Solving matrix with SQMR...\n");
							sqmr(max_iter, minres_tol);
						}
                    }
            
                    // 1. apply P
                    for (int i = 0; i < A.n_cols(); i++) {
                        tmp[perm[i]] = sol_vec[i];
                    }
                    sol_vec = tmp;
                    
                    // 2. apply S
                    for (int i = 0; i < A.n_cols(); i++) {
                        sol_vec[i] = A.S[i]*sol_vec[i];
                    }
                    dif = clock() - start;
                    if (msg_lvl) printf("Solve time:\t%.3f seconds.\n", dif/CLOCKS_PER_SEC);
                    if (msg_lvl) printf("\n");
        
		if (save_sol) {            
                    // save results
                    // TODO: refactor this to be in its own method
                    if (msg_lvl) printf("Solution saved to output_matrices/outsol.mtx.\n");
                    save_vector(sol_vec, "output_matrices/outsol.mtx");
                }

		}
			}
		}
		
		/*! \brief Applies minres on A, preconditioning with factors L and D.
			
			\param max_iter the maximum number of minres iterations.
			\param stop_tol the stopping tolerance of minres. i.e. we stop as soon as the residual goes below stop_tol.
			\param shift shifts A by shift*(identity matrix) to make it more positive definite. This sometimes helps.
		*/
                int minres(int max_iter = 1000, double stop_tol = 1e-6, double shift = 0.0);
		
		/*! \brief Applies SMQR on A, preconditioning with factors L and D.
			
			\param max_iter the maximum number of minres iterations.
			\param stop_tol the stopping tolerance of minres. i.e. we stop as soon as the residual goes below stop_tol.
		*/
                int sqmr(int max_iter = 1000, double stop_tol = 1e-6);
		
		/*! \brief Save results of factorization (automatically saved into the output_matrices folder).
			
			The names of the output matrices follow the format out{}.mtx, where {} describes what the file contains (i.e. A, L, or D).
		*/
		void save() { // TODO: refactor this as a "save factors" method
			if (msg_lvl) cout << "Saving matrices..." << endl;
            if (!perform_inplace) {
                A.save("output_matrices/outB.mtx", true);
                L.save("output_matrices/outL.mtx", false);
            } else {
                A.save("output_matrices/outL.mtx", false);
            }
            
			A.S.save("output_matrices/outS.mtx");
			save_vector(perm, "output_matrices/outP.mtx");
			
			D.save("output_matrices/outD.mtx");
			if (msg_lvl) cout << "Save complete." << endl;
		}
		
		/*! \brief Prints the L and D factors to stdout.
		*/
		void display() {
#ifdef SYM_ILDL_DEBUG
            if (perform_inplace) {
                cout << A << endl;
            } else {
                cout << L << endl;
            }
			cout << D << endl;
			cout << perm << endl;
#endif
		}
};

#include "solver_minres.h"
#include "solver_sqmr.h"

}

#endif
