/**
 * @file    Ordering.cpp
 * @brief   Ordering
 * @author  Christian Potthast
 */

#include <iostream>
#include <string.h> //Added for linux compatibility
#include "Ordering.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
void Ordering::print() const {
  std::cout << "Ordering:" << std::endl;
  printf("Ordering size: %d \n", (int)this->size());
  
  for(size_t i = 0; i < this->size(); i++)
    cout << (*this)[i] << " ";

  cout << endl;
}

/* ************************************************************************* */
bool Ordering::equals(Ordering &ord){	
	if(this->size() != ord.size())
		return false;
	
	vector<std::string>::iterator key;
	vector<std::string>::iterator ord_key;
	for(key = this->begin(); key != this->end(); key++){
		for(ord_key = ord.begin(); ord_key != ord.end(); ord_key++){
			if(strcmp((*key).c_str(), (*ord_key).c_str()) == 0)
				break;
			if(key == this->end())
			  return false;
		}
	}
	return true;
}
/* ************************************************************************* */



