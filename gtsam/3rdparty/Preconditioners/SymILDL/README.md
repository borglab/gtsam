sym-ildl
========

sym-ildl is a C++ package for producing fast incomplete factorizations of symmetric indefinite matrices. Given an n x n symmetric indefinite matrix A, this package produces an incomplete LDL' factorization. To improve stability, the matrix is equilibriated in the max-norm and preordered using the Reverse Cuthill-McKee algorithm prior to factorization. To maintain stability, we use Bunch-Kaufman partial pivoting during the factorization process.

More details as well as extensive documentation can be found at <a href="http://www.cs.ubc.ca/~inutard/html/index.html">http://www.cs.ubc.ca/~inutard/html/index.html</a>.

### Authors: <a href="http://www.cs.ubc.ca/~greif/">Chen Greif</a>, Shiwen He, <a href="https://cs.stanford.edu/people/paulliu/">Paul Liu</a>

<a href="https://cs.stanford.edu/people/paulliu/sym-ildl/html/index.html">Quick Start</a>
=======================================================================