// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2012 Désiré Nuentsa-Wakam <desire.nuentsa_wakam@inria.fr>
//
// Eigen is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Alternatively, you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// Eigen is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License and a copy of the GNU General Public License along with
// Eigen. If not, see <http://www.gnu.org/licenses/>.


#include <iostream>
#include <fstream>
#include "Eigen/SparseCore"
#include <bench/BenchTimer.h>
#include <cstdlib>
#include <string>
#include <Eigen/Cholesky>
#include <Eigen/Jacobi>
#include <Eigen/Householder>
#include <Eigen/IterativeLinearSolvers>
#include <unsupported/Eigen/IterativeSolvers>
#include <Eigen/LU>
#include <unsupported/Eigen/SparseExtra>

#ifdef EIGEN_CHOLMOD_SUPPORT
#include <Eigen/CholmodSupport>
#endif

#ifdef EIGEN_UMFPACK_SUPPORT
#include <Eigen/UmfPackSupport>
#endif

#ifdef EIGEN_PARDISO_SUPPORT
#include <Eigen/PardisoSupport>
#endif

#ifdef EIGEN_SUPERLU_SUPPORT
#include <Eigen/SuperLUSupport>
#endif

#ifdef EIGEN_PASTIX_SUPPORT
#include <Eigen/PaStiXSupport>
#endif

// CONSTANTS
#define EIGEN_UMFPACK  0
#define EIGEN_SUPERLU  1
#define EIGEN_PASTIX  2
#define EIGEN_PARDISO  3
#define EIGEN_BICGSTAB  4
#define EIGEN_BICGSTAB_ILUT  5
#define EIGEN_GMRES 6
#define EIGEN_GMRES_ILUT 7
#define EIGEN_SIMPLICIAL_LDLT  8
#define EIGEN_CHOLMOD_LDLT  9
#define EIGEN_PASTIX_LDLT  10
#define EIGEN_PARDISO_LDLT  11
#define EIGEN_SIMPLICIAL_LLT  12
#define EIGEN_CHOLMOD_SUPERNODAL_LLT  13
#define EIGEN_CHOLMOD_SIMPLICIAL_LLT  14
#define EIGEN_PASTIX_LLT  15
#define EIGEN_PARDISO_LLT  16
#define EIGEN_CG  17
#define EIGEN_CG_PRECOND  18
#define EIGEN_ALL_SOLVERS  19

using namespace Eigen;
using namespace std; 

struct Stats{
  ComputationInfo info;
  double total_time;
  double compute_time;
  double solve_time; 
  double rel_error;
  int memory_used; 
  int iterations;
  int isavail; 
  int isIterative;
}; 

// Global variables for input parameters
int MaximumIters; // Maximum number of iterations
double RelErr; // Relative error of the computed solution

template<typename T> inline typename NumTraits<T>::Real test_precision() { return NumTraits<T>::dummy_precision(); }
template<> inline float test_precision<float>() { return 1e-3f; }                                                             
template<> inline double test_precision<double>() { return 1e-6; }                                                            
template<> inline float test_precision<std::complex<float> >() { return test_precision<float>(); }
template<> inline double test_precision<std::complex<double> >() { return test_precision<double>(); }

void printStatheader(std::ofstream& out)
{
  int LUcnt = 0; 
  string LUlist =" ", LLTlist = "<TH > LLT", LDLTlist = "<TH > LDLT ";
  
#ifdef EIGEN_UMFPACK_SUPPORT
  LUlist += "<TH > UMFPACK "; LUcnt++;
#endif
#ifdef EIGEN_SUPERLU_SUPPORT
  LUlist += "<TH > SUPERLU "; LUcnt++;
#endif
#ifdef EIGEN_CHOLMOD_SUPPORT
  LLTlist += "<TH > CHOLMOD SP LLT<TH > CHOLMOD LLT"; 
  LDLTlist += "<TH>CHOLMOD LDLT"; 
#endif
#ifdef EIGEN_PARDISO_SUPPORT
  LUlist += "<TH > PARDISO LU";  LUcnt++;
  LLTlist += "<TH > PARDISO LLT"; 
  LDLTlist += "<TH > PARDISO LDLT";
#endif
#ifdef EIGEN_PASTIX_SUPPORT
  LUlist += "<TH > PASTIX LU";  LUcnt++;
  LLTlist += "<TH > PASTIX LLT"; 
  LDLTlist += "<TH > PASTIX LDLT";
#endif
  
  out << "<TABLE border=\"1\" >\n ";
  out << "<TR><TH>Matrix <TH> N <TH> NNZ <TH> ";
  if (LUcnt) out << LUlist;
  out << " <TH >BiCGSTAB <TH >BiCGSTAB+ILUT"<< "<TH >GMRES+ILUT" <<LDLTlist << LLTlist <<  "<TH> CG "<< std::endl;
}


template<typename Solver, typename Scalar>
Stats call_solver(Solver &solver, const typename Solver::MatrixType& A, const Matrix<Scalar, Dynamic, 1>& b, const Matrix<Scalar, Dynamic, 1>& refX)
{
  Stats stat; 
  Matrix<Scalar, Dynamic, 1> x; 
  BenchTimer timer; 
  timer.reset();
  timer.start();
  solver.compute(A); 
  if (solver.info() != Success)
  {
    stat.info = NumericalIssue;
    std::cerr << "Solver failed ... \n";
    return stat;
  }
  timer.stop(); 
  stat.compute_time = timer.value();
  
  timer.reset();
  timer.start();
  x = solver.solve(b); 
  if (solver.info() == NumericalIssue)
  {
    stat.info = NumericalIssue;
    std::cerr << "Solver failed ... \n";
    return stat;
  }
  
  timer.stop();
  stat.solve_time = timer.value();
  stat.total_time = stat.solve_time + stat.compute_time;
  stat.memory_used = 0; 
  // Verify the relative error
  if(refX.size() != 0)
    stat.rel_error = (refX - x).norm()/refX.norm();
  else 
  {
    // Compute the relative residual norm
    Matrix<Scalar, Dynamic, 1> temp; 
    temp = A * x; 
    stat.rel_error = (b-temp).norm()/b.norm();
  }
  if ( stat.rel_error > RelErr )
  {
    stat.info = NoConvergence; 
    return stat;
  }
  else 
  {
    stat.info = Success;
    return stat; 
  }
}

template<typename Solver, typename Scalar>
Stats call_directsolver(Solver& solver, const typename Solver::MatrixType& A, const Matrix<Scalar, Dynamic, 1>& b, const Matrix<Scalar, Dynamic, 1>& refX)
{
    Stats stat;
    stat = call_solver(solver, A, b, refX);
    return stat;
}

template<typename Solver, typename Scalar>
Stats call_itersolver(Solver &solver, const typename Solver::MatrixType& A, const Matrix<Scalar, Dynamic, 1>& b, const Matrix<Scalar, Dynamic, 1>& refX)
{
  Stats stat;
  solver.setTolerance(RelErr); 
  solver.setMaxIterations(MaximumIters);
  
  stat = call_solver(solver, A, b, refX); 
  stat.iterations = solver.iterations();
  return stat; 
}

inline void printStatItem(Stats *stat, int solver_id, int& best_time_id, double& best_time_val)
{
  stat[solver_id].isavail = 1;  
  
  if (stat[solver_id].info == NumericalIssue)
  {
    cout << " SOLVER FAILED ... Probably a numerical issue \n";
    return;
  }
  if (stat[solver_id].info == NoConvergence){
    cout << "REL. ERROR " <<  stat[solver_id].rel_error;
    if(stat[solver_id].isIterative == 1)
      cout << " (" << stat[solver_id].iterations << ") \n"; 
    return;
  }
  
  // Record the best CPU time 
  if (!best_time_val) 
  {
    best_time_val = stat[solver_id].total_time;
    best_time_id = solver_id;
  }
  else if (stat[solver_id].total_time < best_time_val)
  {
    best_time_val = stat[solver_id].total_time;
    best_time_id = solver_id; 
  }
  // Print statistics to standard output
  if (stat[solver_id].info == Success){
    cout<< "COMPUTE TIME : " << stat[solver_id].compute_time<< " \n";
    cout<< "SOLVE TIME : " << stat[solver_id].solve_time<< " \n";
    cout<< "TOTAL TIME : " << stat[solver_id].total_time<< " \n";
    cout << "REL. ERROR : " << stat[solver_id].rel_error ;
    if(stat[solver_id].isIterative == 1) {
      cout << " (" << stat[solver_id].iterations << ") ";
    }
    cout << std::endl;
  }
    
}


/* Print the results from all solvers corresponding to a particular matrix 
 * The best CPU time is printed in bold
 */
inline void printHtmlStatLine(Stats *stat, int best_time_id, string& statline)
{
  
  string markup;
  ostringstream compute,solve,total,error;
  for (int i = 0; i < EIGEN_ALL_SOLVERS; i++) 
  {
    if (stat[i].isavail == 0) continue;
    if(i == best_time_id)
      markup = "<TD style=\"background-color:red\">";
    else
      markup = "<TD>";
    
    if (stat[i].info == Success){
      compute << markup << stat[i].compute_time;
      solve << markup << stat[i].solve_time;
      total << markup << stat[i].total_time; 
      error << " <TD> " << stat[i].rel_error;
      if(stat[i].isIterative == 1) {
        error << " (" << stat[i].iterations << ") ";
      }
    }
    else {
      compute << " <TD> -" ;
      solve << " <TD> -" ;
      total << " <TD> -" ;
      if(stat[i].info == NoConvergence){
        error << " <TD> "<< stat[i].rel_error ;
        if(stat[i].isIterative == 1)
          error << " (" << stat[i].iterations << ") "; 
      }
      else    error << " <TD> - ";
    }
  }
  
  statline = "<TH>Compute Time " + compute.str() + "\n" 
                        +  "<TR><TH>Solve Time " + solve.str() + "\n" 
                        +  "<TR><TH>Total Time " + total.str() + "\n" 
                        +"<TR><TH>Error(Iter)" + error.str() + "\n"; 
  
}

template <typename Scalar>
int SelectSolvers(const SparseMatrix<Scalar>&A, unsigned int sym, Matrix<Scalar, Dynamic, 1>& b, const Matrix<Scalar, Dynamic, 1>& refX, Stats *stat)
{
  typedef SparseMatrix<Scalar, ColMajor> SpMat; 
  // First, deal with Nonsymmetric and symmetric matrices
  int best_time_id = 0; 
  double best_time_val = 0.0;
  //UMFPACK
  #ifdef EIGEN_UMFPACK_SUPPORT
  {
    cout << "Solving with UMFPACK LU ... \n"; 
    UmfPackLU<SpMat> solver; 
    stat[EIGEN_UMFPACK] = call_directsolver(solver, A, b, refX); 
    printStatItem(stat, EIGEN_UMFPACK, best_time_id, best_time_val); 
  }
  #endif
    //SuperLU
  #ifdef EIGEN_SUPERLU_SUPPORT
  {
    cout << "\nSolving with SUPERLU ... \n"; 
    SuperLU<SpMat> solver;
    stat[EIGEN_SUPERLU] = call_directsolver(solver, A, b, refX); 
    printStatItem(stat, EIGEN_SUPERLU, best_time_id, best_time_val); 
  }
  #endif
    
   // PaStix LU
  #ifdef EIGEN_PASTIX_SUPPORT
  {
    cout << "\nSolving with PASTIX LU ... \n"; 
    PastixLU<SpMat> solver; 
    stat[EIGEN_PASTIX] = call_directsolver(solver, A, b, refX) ;
    printStatItem(stat, EIGEN_PASTIX, best_time_id, best_time_val); 
  }
  #endif

   //PARDISO LU
  #ifdef EIGEN_PARDISO_SUPPORT
  {
    cout << "\nSolving with PARDISO LU ... \n"; 
    PardisoLU<SpMat>  solver; 
    stat[EIGEN_PARDISO] = call_directsolver(solver, A, b, refX);
    printStatItem(stat, EIGEN_PARDISO, best_time_id, best_time_val); 
  }
  #endif


  
  //BiCGSTAB
  {
    cout << "\nSolving with BiCGSTAB ... \n"; 
    BiCGSTAB<SpMat> solver; 
    stat[EIGEN_BICGSTAB] = call_itersolver(solver, A, b, refX);
    stat[EIGEN_BICGSTAB].isIterative = 1;
    printStatItem(stat, EIGEN_BICGSTAB, best_time_id, best_time_val); 
  }
  //BiCGSTAB+ILUT
  {
    cout << "\nSolving with BiCGSTAB and ILUT ... \n"; 
    BiCGSTAB<SpMat, IncompleteLUT<Scalar> > solver; 
    stat[EIGEN_BICGSTAB_ILUT] = call_itersolver(solver, A, b, refX);
    stat[EIGEN_BICGSTAB_ILUT].isIterative = 1;
    printStatItem(stat, EIGEN_BICGSTAB_ILUT, best_time_id, best_time_val); 
  }
  
   
  //GMRES
//   {
//     cout << "\nSolving with GMRES ... \n"; 
//     GMRES<SpMat> solver; 
//     stat[EIGEN_GMRES] = call_itersolver(solver, A, b, refX);
//     stat[EIGEN_GMRES].isIterative = 1;
//     printStatItem(stat, EIGEN_GMRES, best_time_id, best_time_val); 
//   }
  //GMRES+ILUT
  {
    cout << "\nSolving with GMRES and ILUT ... \n"; 
    GMRES<SpMat, IncompleteLUT<Scalar> > solver; 
    stat[EIGEN_GMRES_ILUT] = call_itersolver(solver, A, b, refX);
    stat[EIGEN_GMRES_ILUT].isIterative = 1;
    printStatItem(stat, EIGEN_GMRES_ILUT, best_time_id, best_time_val); 
  }
  
  // Hermitian and not necessarily positive-definites
  if (sym != NonSymmetric)
  {
    // Internal Cholesky
    {
      cout << "\nSolving with Simplicial LDLT ... \n"; 
      SimplicialLDLT<SpMat, Lower> solver;
      stat[EIGEN_SIMPLICIAL_LDLT] = call_directsolver(solver, A, b, refX); 
      printStatItem(stat, EIGEN_SIMPLICIAL_LDLT, best_time_id, best_time_val); 
    }
    
    // CHOLMOD
    #ifdef EIGEN_CHOLMOD_SUPPORT
    {
      cout << "\nSolving with CHOLMOD LDLT ... \n"; 
      CholmodDecomposition<SpMat, Lower> solver;
      solver.setMode(CholmodLDLt);
      stat[EIGEN_CHOLMOD_LDLT] =  call_directsolver(solver, A, b, refX);
      printStatItem(stat,EIGEN_CHOLMOD_LDLT, best_time_id, best_time_val); 
    }
    #endif
    
    //PASTIX LLT
    #ifdef EIGEN_PASTIX_SUPPORT
    {
      cout << "\nSolving with PASTIX LDLT ... \n"; 
      PastixLDLT<SpMat, Lower> solver; 
      stat[EIGEN_PASTIX_LDLT] = call_directsolver(solver, A, b, refX);
      printStatItem(stat,EIGEN_PASTIX_LDLT, best_time_id, best_time_val); 
    }
    #endif
    
    //PARDISO LLT
    #ifdef EIGEN_PARDISO_SUPPORT
    {
      cout << "\nSolving with PARDISO LDLT ... \n"; 
      PardisoLDLT<SpMat, Lower> solver; 
      stat[EIGEN_PARDISO_LDLT] = call_directsolver(solver, A, b, refX); 
      printStatItem(stat,EIGEN_PARDISO_LDLT, best_time_id, best_time_val); 
    }
    #endif
  }

   // Now, symmetric POSITIVE DEFINITE matrices
  if (sym == SPD)
  {
    
    //Internal Sparse Cholesky
    {
      cout << "\nSolving with SIMPLICIAL LLT ... \n"; 
      SimplicialLLT<SpMat, Lower> solver; 
      stat[EIGEN_SIMPLICIAL_LLT] = call_directsolver(solver, A, b, refX); 
      printStatItem(stat,EIGEN_SIMPLICIAL_LLT, best_time_id, best_time_val); 
    }
    
    // CHOLMOD
    #ifdef EIGEN_CHOLMOD_SUPPORT
    {
      // CholMOD SuperNodal LLT
      cout << "\nSolving with CHOLMOD LLT (Supernodal)... \n"; 
      CholmodDecomposition<SpMat, Lower> solver;
      solver.setMode(CholmodSupernodalLLt);
      stat[EIGEN_CHOLMOD_SUPERNODAL_LLT] = call_directsolver(solver, A, b, refX);
      printStatItem(stat,EIGEN_CHOLMOD_SUPERNODAL_LLT, best_time_id, best_time_val); 
      // CholMod Simplicial LLT
      cout << "\nSolving with CHOLMOD LLT (Simplicial) ... \n"; 
      solver.setMode(CholmodSimplicialLLt);
      stat[EIGEN_CHOLMOD_SIMPLICIAL_LLT] = call_directsolver(solver, A, b, refX);
      printStatItem(stat,EIGEN_CHOLMOD_SIMPLICIAL_LLT, best_time_id, best_time_val); 
    }
    #endif
    
    //PASTIX LLT
    #ifdef EIGEN_PASTIX_SUPPORT
    {
      cout << "\nSolving with PASTIX LLT ... \n"; 
      PastixLLT<SpMat, Lower> solver; 
      stat[EIGEN_PASTIX_LLT] =  call_directsolver(solver, A, b, refX);
      printStatItem(stat,EIGEN_PASTIX_LLT, best_time_id, best_time_val); 
    }
    #endif
    
    //PARDISO LLT
    #ifdef EIGEN_PARDISO_SUPPORT
    {
      cout << "\nSolving with PARDISO LLT ... \n"; 
      PardisoLLT<SpMat, Lower> solver; 
      stat[EIGEN_PARDISO_LLT] = call_directsolver(solver, A, b, refX);
      printStatItem(stat,EIGEN_PARDISO_LLT, best_time_id, best_time_val); 
    }
    #endif
    
    // Internal CG
    {
      cout << "\nSolving with CG ... \n"; 
      ConjugateGradient<SpMat, Lower> solver; 
      stat[EIGEN_CG] = call_itersolver(solver, A, b, refX);
      stat[EIGEN_CG].isIterative = 1;
      printStatItem(stat,EIGEN_CG, best_time_id, best_time_val); 
    }
    //CG+IdentityPreconditioner
//     {
//       cout << "\nSolving with CG and IdentityPreconditioner ... \n"; 
//       ConjugateGradient<SpMat, Lower, IdentityPreconditioner> solver; 
//       stat[EIGEN_CG_PRECOND] = call_itersolver(solver, A, b, refX);
//       stat[EIGEN_CG_PRECOND].isIterative = 1;
//       printStatItem(stat,EIGEN_CG_PRECOND, best_time_id, best_time_val); 
//     }
  } // End SPD matrices 
  
  return best_time_id;
}

/* Browse all the matrices available in the specified folder 
 * and solve the associated linear system.
 * The results of each solve are printed in the standard output
 * and optionally in the provided html file
 */
template <typename Scalar>
void Browse_Matrices(const string folder, bool statFileExists, std::string& statFile, int maxiters, double tol)
{
  MaximumIters = maxiters; // Maximum number of iterations, global variable 
  RelErr = tol;  //Relative residual error  as stopping criterion for iterative solvers
  MatrixMarketIterator<Scalar> it(folder);
  Stats stat[EIGEN_ALL_SOLVERS];
  for ( ; it; ++it)
  {    
    for (int i = 0; i < EIGEN_ALL_SOLVERS; i++)
    {
      stat[i].isavail = 0;
      stat[i].isIterative = 0;
    }
    
    int best_time_id;
    cout<< "\n\n===================================================== \n";
    cout<< " ======  SOLVING WITH MATRIX " << it.matname() << " ====\n";
    cout<< " =================================================== \n\n";
    Matrix<Scalar, Dynamic, 1> refX;
    if(it.hasrefX()) refX = it.refX();
    best_time_id = SelectSolvers<Scalar>(it.matrix(), it.sym(), it.rhs(), refX, &stat[0]);
    
    if(statFileExists)
    {
      string statline;
      printHtmlStatLine(&stat[0], best_time_id, statline); 
      std::ofstream statbuf(statFile.c_str(), std::ios::app);
      statbuf << "<TR><TH rowspan=\"4\">" << it.matname() << " <TD rowspan=\"4\"> "
      << it.matrix().rows() << " <TD rowspan=\"4\"> " << it.matrix().nonZeros()<< " "<< statline ;
      statbuf.close();
    }
  } 
} 

bool get_options(int argc, char **args, string option, string* value=0)
{
  int idx = 1, found=false; 
  while (idx<argc && !found){
    if (option.compare(args[idx]) == 0){
      found = true; 
      if(value) *value = args[idx+1];
    }
    idx+=2;
  }
  return found; 
}
