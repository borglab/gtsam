/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeVirtual.cpp
 * @brief   Time the overhead of using virtual destructors and methods
 * @author  Richard Roberts
 * @date    Nov 6, 2011
 */

#include <gtsam/base/timing.h>

#include <boost/shared_ptr.hpp>
#include <boost/intrusive_ptr.hpp>

#include <iostream>

using namespace std;
using namespace boost;

struct DtorTestBase {
  DtorTestBase() { cout << "  DtorTestBase" << endl; }
  virtual ~DtorTestBase() { cout << "  ~DtorTestBase" << endl; }
};

struct DtorTestDerived : public DtorTestBase {
  DtorTestDerived() { cout << "  DtorTestDerived" << endl; }
  virtual ~DtorTestDerived() { cout << "  ~DtorTestDerived" << endl; }
};


struct VirtualBase {
  VirtualBase() { }
  virtual void method() = 0;
  virtual ~VirtualBase() { }
};

struct VirtualDerived : public VirtualBase {
  double data;
  VirtualDerived() { data = rand(); }
  virtual void method() { data = rand(); }
  virtual ~VirtualDerived() { }
};

struct NonVirtualBase {
  NonVirtualBase() { }
  ~NonVirtualBase() { }
};

struct NonVirtualDerived : public NonVirtualBase {
  double data;
  NonVirtualDerived() { data = rand(); }
  void method() { data = rand(); }
  ~NonVirtualDerived() { }
};


int main(int argc, char *argv[]) {

  // Virtual destructor test
  cout << "Stack objects:" << endl;
  cout << "Base:" << endl;
  { DtorTestBase b; }
  cout << "Derived:" << endl;
  { DtorTestDerived d; }

  cout << "Heap objects:" << endl;
  cout << "Base:" << endl;
  { DtorTestBase *b = new DtorTestBase(); delete b; }
  cout << "Derived:" << endl;
  { DtorTestDerived *d = new DtorTestDerived(); delete d; }
  cout << "Derived with base pointer:" << endl;
  { DtorTestBase *b = new DtorTestDerived(); delete b; }

  int n = 10000000;

  {
  VirtualBase** b = new VirtualBase*[n];
  tic_(Virtual);
  tic_(new);
  for(int i=0; i<n; ++i)
    b[i] = new VirtualDerived();
  toc_(new);
  tic_(method);
  for(int i=0; i<n; ++i)
    b[i]->method();
  toc_(method);
  tic_(dynamic_cast);
  for(int i=0; i<n; ++i) {
    VirtualDerived* d = dynamic_cast<VirtualDerived*>(b[i]);
    if(d)
      d->method();
  }
  toc_(dynamic_cast);
  tic_(delete);
  for(int i=0; i<n; ++i)
    delete b[i];
  toc_(delete);
  toc_(Virtual);
  delete[] b;
  }


  {
  NonVirtualDerived** d = new NonVirtualDerived*[n];
  tic_(NonVirtual);
  tic_(new);
  for(int i=0; i<n; ++i)
    d[i] = new NonVirtualDerived();
  toc_(new);
  tic_(method);
  for(int i=0; i<n; ++i)
    d[i]->method();
  toc_(method);
  tic_(dynamic_cast (does nothing));
  for(int i=0; i<n; ++i)
    d[i]->method();
  toc_(dynamic_cast (does nothing));
  tic_(delete);
  for(int i=0; i<n; ++i)
    delete d[i];
  toc_(delete);
  toc_(NonVirtual);
  delete[] d;
  }

  tictoc_finishedIteration_();
  tictoc_print_();

  return 0;
}
