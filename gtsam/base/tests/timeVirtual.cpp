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
 * @date    Dec 3, 2010
 */

#include <gtsam/base/timing.h>

#include <boost/shared_ptr.hpp>
#include <boost/intrusive_ptr.hpp>

#include <iostream>

using namespace std;
using namespace boost;

struct Plain {
  size_t data;
  Plain(size_t _data) : data(_data) {}
  void setData(size_t data) { this->data = data; }
};

struct Virtual {
  size_t data;
  Virtual(size_t _data) : data(_data) {}
  virtual void setData(size_t data) { this->data = data; }
  virtual ~Virtual() {}
};

struct VirtualCounted {
  size_t data;
  size_t refCount;
  VirtualCounted(size_t _data) : data(_data) {}
  virtual void setData(size_t data) { this->data = data; }
  virtual ~VirtualCounted() {}
};

void intrusive_ptr_add_ref(VirtualCounted* obj) { ++ obj->refCount; }
void intrusive_ptr_release(VirtualCounted* obj) {
  assert(obj->refCount > 0);
  -- obj->refCount;
  if(obj->refCount == 0)
    delete obj;
}

int main(int argc, char *argv[]) {

  size_t trials = 10000000;

  tic_("heap plain alloc, dealloc");
  for(size_t i=0; i<trials; ++i) {
    Plain *obj = new Plain(i);
    delete obj;
  }
  toc_("heap plain alloc, dealloc");

  tic_("heap virtual alloc, dealloc");
  for(size_t i=0; i<trials; ++i) {
    Virtual *obj = new Virtual(i);
    delete obj;
  }
  toc_("heap virtual alloc, dealloc");

  tic_("stack plain alloc, dealloc");
  for(size_t i=0; i<trials; ++i) {
    Plain obj(i);
  }
  toc_("stack plain alloc, dealloc");

  tic_("stack virtual alloc, dealloc");
  for(size_t i=0; i<trials; ++i) {
    Virtual obj(i);
  }
  toc_("stack virtual alloc, dealloc");

  tic_("shared plain alloc, dealloc");
  for(size_t i=0; i<trials; ++i) {
    boost::shared_ptr<Plain> obj(new Plain(i));
  }
  toc_("shared plain alloc, dealloc");

  tic_("shared virtual alloc, dealloc");
  for(size_t i=0; i<trials; ++i) {
    boost::shared_ptr<Virtual> obj(new Virtual(i));
  }
  toc_("shared virtual alloc, dealloc");


  tic_("heap plain alloc, dealloc, call");
  for(size_t i=0; i<trials; ++i) {
    Plain *obj = new Plain(i);
    obj->setData(i+1);
    delete obj;
  }
  toc_("heap plain alloc, dealloc, call");

  tic_("heap virtual alloc, dealloc, call");
  for(size_t i=0; i<trials; ++i) {
    Virtual *obj = new Virtual(i);
    obj->setData(i+1);
    delete obj;
  }
  toc_("heap virtual alloc, dealloc, call");

  tic_("stack plain alloc, dealloc, call");
  for(size_t i=0; i<trials; ++i) {
    Plain obj(i);
    obj.setData(i+1);
  }
  toc_("stack plain alloc, dealloc, call");

  tic_("stack virtual alloc, dealloc, call");
  for(size_t i=0; i<trials; ++i) {
    Virtual obj(i);
    obj.setData(i+1);
  }
  toc_("stack virtual alloc, dealloc, call");

  tic_("shared plain alloc, dealloc, call");
  for(size_t i=0; i<trials; ++i) {
    boost::shared_ptr<Plain> obj(new Plain(i));
    obj->setData(i+1);
  }
  toc_("shared plain alloc, dealloc, call");

  tic_("shared virtual alloc, dealloc, call");
  for(size_t i=0; i<trials; ++i) {
    boost::shared_ptr<Virtual> obj(new Virtual(i));
    obj->setData(i+1);
  }
  toc_("shared virtual alloc, dealloc, call");

  tic_("intrusive virtual alloc, dealloc, call");
  for(size_t i=0; i<trials; ++i) {
    intrusive_ptr<VirtualCounted> obj(new VirtualCounted(i));
    obj->setData(i+1);
  }
  toc_("intrusive virtual alloc, dealloc, call");

  tictoc_print_();

  return 0;
}

