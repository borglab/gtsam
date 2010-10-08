/**
 * @file    timeublas.cpp
 * @brief   Tests to help determine which way of accomplishing something in ublas is faster
 * @author  Richard Roberts
 * @created Sep 18, 2010
 */

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/random.hpp>
#include <boost/timer.hpp>
#include <boost/format.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/foreach.hpp>

#include <iostream>
#include <vector>
#include <utility>

using namespace std;
namespace ublas = boost::numeric::ublas;
using boost::timer;
using boost::format;
using namespace boost::lambda;

static boost::variate_generator<boost::mt19937, boost::uniform_real<> > rng(boost::mt19937(), boost::uniform_real<>(-1.0, 0.0));
typedef ublas::matrix<double> matrix;
typedef ublas::matrix_range<matrix> matrix_range;
using ublas::range;
using ublas::triangular_matrix;

int main(int argc, char* argv[]) {

  if(false) {
    cout << "\nTiming matrix_range:" << endl;

    // We use volatile here to make these appear to the optimizing compiler as
    // if their values are only known at run-time.
    volatile size_t m=500;
    volatile size_t n=300;
    volatile size_t nReps = 1000;
    assert(m > n);
    boost::variate_generator<boost::mt19937, boost::uniform_int<size_t> > rni(boost::mt19937(), boost::uniform_int<size_t>(0,m-1));
    boost::variate_generator<boost::mt19937, boost::uniform_int<size_t> > rnj(boost::mt19937(), boost::uniform_int<size_t>(0,n-1));
    matrix mat(m,n);
    matrix_range full(mat, range(0,m), range(0,n));
    matrix_range top(mat, range(0,n), range(0,n));
    matrix_range block(mat, range(m/4, m-m/4), range(n/4, n-n/4));

    cout << format("  Basic: %1%x%2%\n") % m % n;
    cout << format("  Full:  mat(%1%:%2%, %3%:%4%)\n") % 0 % m % 0 % n;
    cout << format("  Top:   mat(%1%:%2%, %3%:%4%)\n") % 0 % n % 0 % n;
    cout << format("  Block: mat(%1%:%2%, %3%:%4%)\n") % size_t(m/4) % size_t(m-m/4) % size_t(n/4) % size_t(n-n/4);
    cout << endl;

    {
      timer tim;
      double basicTime, fullTime, topTime, blockTime;

      cout << "Row-major matrix, row-major assignment:" << endl;

      // Do a few initial assignments to let any cache effects stabilize
      for(size_t rep=0; rep<1000; ++rep)
        for(size_t i=0; i<mat.size1(); ++i)
          for(size_t j=0; j<mat.size2(); ++j)
            mat(i,j) = rng();

      tim.restart();
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t i=0; i<mat.size1(); ++i)
          for(size_t j=0; j<mat.size2(); ++j)
            mat(i,j) = rng();
      basicTime = tim.elapsed();
      cout << format("  Basic: %1% mus/element") % double(1000000 * basicTime / double(mat.size1()*mat.size2()*nReps)) << endl;

      tim.restart();
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t i=0; i<full.size1(); ++i)
          for(size_t j=0; j<full.size2(); ++j)
            full(i,j) = rng();
      fullTime = tim.elapsed();
      cout << format("  Full:  %1% mus/element") % double(1000000 * fullTime / double(full.size1()*full.size2()*nReps)) << endl;

      tim.restart();
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t i=0; i<top.size1(); ++i)
          for(size_t j=0; j<top.size2(); ++j)
            top(i,j) = rng();
      topTime = tim.elapsed();
      cout << format("  Top:   %1% mus/element") % double(1000000 * topTime / double(top.size1()*top.size2()*nReps)) << endl;

      tim.restart();
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t i=0; i<block.size1(); ++i)
          for(size_t j=0; j<block.size2(); ++j)
            block(i,j) = rng();
      blockTime = tim.elapsed();
      cout << format("  Block: %1% mus/element") % double(1000000 * blockTime / double(block.size1()*block.size2()*nReps)) << endl;

      cout << endl;
    }

    {
      timer tim;
      double basicTime, fullTime, topTime, blockTime;

      cout << "Row-major matrix, column-major assignment:" << endl;

      // Do a few initial assignments to let any cache effects stabilize
      for(size_t rep=0; rep<1000; ++rep)
        for(size_t j=0; j<mat.size2(); ++j)
          for(size_t i=0; i<mat.size1(); ++i)
            mat(i,j) = rng();

      tim.restart();
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t j=0; j<mat.size2(); ++j)
          for(size_t i=0; i<mat.size1(); ++i)
            mat(i,j) = rng();
      basicTime = tim.elapsed();
      cout << format("  Basic: %1% mus/element") % double(1000000 * basicTime / double(mat.size1()*mat.size2()*nReps)) << endl;

      tim.restart();
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t j=0; j<full.size2(); ++j)
          for(size_t i=0; i<full.size1(); ++i)
            full(i,j) = rng();
      fullTime = tim.elapsed();
      cout << format("  Full:  %1% mus/element") % double(1000000 * fullTime / double(full.size1()*full.size2()*nReps)) << endl;

      tim.restart();
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t j=0; j<top.size2(); ++j)
          for(size_t i=0; i<top.size1(); ++i)
            top(i,j) = rng();
      topTime = tim.elapsed();
      cout << format("  Top:   %1% mus/element") % double(1000000 * topTime / double(top.size1()*top.size2()*nReps)) << endl;

      tim.restart();
      for(size_t rep=0; rep<nReps; ++rep)
        for(size_t j=0; j<block.size2(); ++j)
          for(size_t i=0; i<block.size1(); ++i)
            block(i,j) = rng();
      blockTime = tim.elapsed();
      cout << format("  Block: %1% mus/element") % double(1000000 * blockTime / double(block.size1()*block.size2()*nReps)) << endl;

      cout << endl;
    }

    {
      timer tim;
      double basicTime, fullTime, topTime, blockTime;
      typedef pair<size_t,size_t> ij_t;
      vector<ij_t> ijs(100000);

      cout << "Row-major matrix, random assignment:" << endl;

      // Do a few initial assignments to let any cache effects stabilize
      for_each(ijs.begin(), ijs.end(), _1 = make_pair(rni(),rnj()));
      for(size_t rep=0; rep<1000; ++rep)
        BOOST_FOREACH(const ij_t& ij, ijs) { mat(ij.first, ij.second) = rng(); }

      for_each(ijs.begin(), ijs.end(), _1 = make_pair(rni(),rnj()));
      for(size_t rep=0; rep<1000; ++rep)
        BOOST_FOREACH(const ij_t& ij, ijs) { mat(ij.first, ij.second) = rng(); }
      basicTime = tim.elapsed();
      cout << format("  Basic: %1% mus/element") % double(1000000 * basicTime / double(ijs.size()*nReps)) << endl;

      for_each(ijs.begin(), ijs.end(), _1 = make_pair(rni(),rnj()));
      for(size_t rep=0; rep<1000; ++rep)
        BOOST_FOREACH(const ij_t& ij, ijs) { full(ij.first, ij.second) = rng(); }
      fullTime = tim.elapsed();
      cout << format("  Full:  %1% mus/element") % double(1000000 * fullTime / double(ijs.size()*nReps)) << endl;

      for_each(ijs.begin(), ijs.end(), _1 = make_pair(rni()%top.size1(),rnj()));
      for(size_t rep=0; rep<1000; ++rep)
        BOOST_FOREACH(const ij_t& ij, ijs) { top(ij.first, ij.second) = rng(); }
      topTime = tim.elapsed();
      cout << format("  Top:   %1% mus/element") % double(1000000 * topTime / double(ijs.size()*nReps)) << endl;

      for_each(ijs.begin(), ijs.end(), _1 = make_pair(rni()%block.size1(),rnj()%block.size2()));
      for(size_t rep=0; rep<1000; ++rep)
        BOOST_FOREACH(const ij_t& ij, ijs) { block(ij.first, ij.second) = rng(); }
      blockTime = tim.elapsed();
      cout << format("  Block: %1% mus/element") % double(1000000 * blockTime / double(ijs.size()*nReps)) << endl;

      cout << endl;
    }
  }

  if(true) {
    cout << "\nTesting square triangular matrices:" << endl;

    typedef triangular_matrix<double, ublas::upper, ublas::column_major> triangular;
    typedef ublas::matrix<double, ublas::column_major> matrix;

    triangular tri(5,5);

    matrix mat(5,5);
    for(size_t j=0; j<mat.size2(); ++j)
      for(size_t i=0; i<mat.size1(); ++i)
        mat(i,j) = rng();

    tri = ublas::triangular_adaptor<matrix, ublas::upper>(mat);
    cout << "  Assigned from triangular adapter: " << tri << endl;

    cout << "  Triangular adapter of mat: " << ublas::triangular_adaptor<matrix, ublas::upper>(mat) << endl;

    for(size_t j=0; j<mat.size2(); ++j)
      for(size_t i=0; i<mat.size1(); ++i)
        mat(i,j) = rng();
    mat = tri;
    cout << "  Assign matrix from triangular: " << mat << endl;

    for(size_t j=0; j<mat.size2(); ++j)
      for(size_t i=0; i<mat.size1(); ++i)
        mat(i,j) = rng();
    (ublas::triangular_adaptor<matrix, ublas::upper>(mat)) = tri;
    cout << "  Assign triangular adaptor from triangular: " << mat << endl;
  }

  {
    cout << "\nTesting wide triangular matrices:" << endl;

    typedef triangular_matrix<double, ublas::upper, ublas::column_major> triangular;
    typedef ublas::matrix<double, ublas::column_major> matrix;

    triangular tri(5,7);

    matrix mat(5,7);
    for(size_t j=0; j<mat.size2(); ++j)
      for(size_t i=0; i<mat.size1(); ++i)
        mat(i,j) = rng();

    tri = ublas::triangular_adaptor<matrix, ublas::upper>(mat);
    cout << "  Assigned from triangular adapter: " << tri << endl;

    cout << "  Triangular adapter of mat: " << ublas::triangular_adaptor<matrix, ublas::upper>(mat) << endl;

    for(size_t j=0; j<mat.size2(); ++j)
      for(size_t i=0; i<mat.size1(); ++i)
        mat(i,j) = rng();
    mat = tri;
    cout << "  Assign matrix from triangular: " << mat << endl;

    for(size_t j=0; j<mat.size2(); ++j)
      for(size_t i=0; i<mat.size1(); ++i)
        mat(i,j) = rng();
    mat = ublas::triangular_adaptor<matrix, ublas::upper>(mat);
    cout << "  Assign matrix from triangular adaptor of self: " << mat << endl;
  }

  {
    cout << "\nTesting subvectors:" << endl;

    typedef ublas::matrix<double, ublas::column_major> matrix;
    matrix mat(4,4);

    for(size_t j=0; j<mat.size2(); ++j)
      for(size_t i=0; i<mat.size1(); ++i)
        mat(i,j) = i*mat.size1() + j;
    cout << "  mat = " << mat;

    cout << "  vec(1:4, 2:2) = " << ublas::matrix_vector_range<matrix>(mat, ublas::range(1,4), ublas::range(2,2));

  }

  return 0;

}
