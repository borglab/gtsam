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

#include <iostream>

using namespace std;
using namespace boost;
using namespace gtsam;

struct DtorTestBase {
  DtorTestBase() { cout << "  DtorTestBase" << endl; }
  virtual ~DtorTestBase() { cout << "  ~DtorTestBase" << endl; }
};

struct DtorTestDerived : public DtorTestBase {
  DtorTestDerived() { cout << "  DtorTestDerived" << endl; }
  ~DtorTestDerived() override { cout << "  ~DtorTestDerived" << endl; }
};


struct VirtualBase {
  VirtualBase() { }
  virtual void method() = 0;
  virtual ~VirtualBase() { }
};

struct VirtualDerived : public VirtualBase {
  double data;
  VirtualDerived() { data = rand(); }
  void method() override { data = rand(); }
  ~VirtualDerived() override { }
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
  gttic_(Virtual);
  gttic_(new);
  for(int i=0; i<n; ++i)
    b[i] = new VirtualDerived();
  gttoc_(new);
  gttic_(method);
  for(int i=0; i<n; ++i)
    b[i]->method();
  gttoc_(method);
  gttic_(dynamic_cast);
  for(int i=0; i<n; ++i) {
    VirtualDerived* d = dynamic_cast<VirtualDerived*>(b[i]);
    if(d)
      d->method();
  }
  gttoc_(dynamic_cast);
  gttic_(delete);
  for(int i=0; i<n; ++i)
    delete b[i];
  gttoc_(delete);
  gttoc_(Virtual);
  delete[] b;
  }


  {
  NonVirtualDerived** d = new NonVirtualDerived*[n];
  gttic_(NonVirtual);
  gttic_(new);
  for(int i=0; i<n; ++i)
    d[i] = new NonVirtualDerived();
  gttoc_(new);
  gttic_(method);
  for(int i=0; i<n; ++i)
    d[i]->method();
  gttoc_(method);
  gttic_(dynamic_cast_does_nothing);
  for(int i=0; i<n; ++i)
    d[i]->method();
  gttoc_(dynamic_cast_does_nothing);
  gttic_(delete);
  for(int i=0; i<n; ++i)
    delete d[i];
  gttoc_(delete);
  gttoc_(NonVirtual);
  delete[] d;
  }

  tictoc_finishedIteration_();
  tictoc_print_();

  return 0;
}
