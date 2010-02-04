/*
 *  Created on: Feb 3, 2010
 *      Author: cbeall3
 */

#include <boost/shared_ptr.hpp>
#include <CppUnitLite/TestHarness.h>
#include "Key.h"

//    type 'a t =
//        Empty
//      | Node of 'a t * key * 'a * 'a t * int

namespace gtsam{

template <class Key, class Value>
struct Node {
	typedef boost::shared_ptr<Node> Tree;
	Tree left_, right_;
	Key key_;
	Value value_;
	size_t height_;
	/**
	 * leaf node with height 1
	 */
	Node(const Key& key, const Value& value)
	:key_(key),value_(value),height_(1) {}
	Node(const Tree& l, const Key& key, const Value& value, const Tree& r, size_t height)
	:left_(l),key_(key),value_(value),right_(r),height_(height) {}
	Node() {}
};

template <class Key, class Value>
size_t height(const typename boost::shared_ptr<Node<Key,Value> >& t) {
	if (t) return t->height_; else return 0;
}

template <class Key, class Value>
typename Node<Key,Value>::Tree create(const typename Node<Key,Value>::Tree& l, const Key& key,
		const Value& value, const typename Node<Key,Value>::Tree& r) {
	size_t hl = height(l), hr = height(r);
	size_t h = hl >= hr ? hl + 1 : hr + 1;
	return typename Node<Key,Value>::Tree(new Node<Key, Value>(l,key,value,r, h));
}

template <class Key, class Value>
typename Node<Key,Value>::Tree bal(const typename Node<Key,Value>::Tree& l, const Key& key,
		const Value& value, const typename Node<Key,Value>::Tree& r) {
	size_t hl = height(l), hr = height(r);
	if(hl > hr+2) {
		if(hl == 0) throw("Left tree is empty");
		else if(l->left_->height_ >= l->right_->height_) {
		  //  create ll lv ld (create lr x d r)
			return create(l->left_,l->key_,l->value_, create(l->right_, key, value, r));
		}
		else{
			if(l->right_->height_ == 0) throw("Left->Right is empty");
			else {
				 // create (create ll lv ld lrl) lrv lrd (create lrr x d r)
				return create(
						create(l->left_,l->key_,l->value_,l->right_->left_),
						l->right_->key_,
						l->right_->value_,
						create(l->right_->right_,key,value,r));
			}
		}
	}
	else if (hr > hl + 2) {
		if(hr == 0) throw("Right tree is empty");
		else if(r->right_->height_ >= r->left_->height_) {
			//  create (create l x d rl) rv rd rr
			return create(create(l,key,value,r->left_),r->key_,r->value_,r);
		}
		else{
			if(r->left_->height_ == 0) throw("Right->Left is empty");
		  else {
			  //  create (create l x d rll) rlv rld (create rlr rv rd rr)
			  return create(
			  		create(l,key,value,r->left_->left_),
			  		r->left_->key_,
			  		r->left_->value_,
			  		create(r->left_->right_,r->key_,r->value_,r->right_));
		  }
		}
	}
	else {
    return create(l,key,value,r);
	}
}

template <class Key, class Value>
typename Node<Key, Value>::Tree add(const Key& key, const Value& value, const typename Node<Key,Value>::Tree& tree) {
	if(tree == NULL) {
		return typename Node<Key,Value>::Tree(new Node<Key,Value>(key, value));
	}
	if(key == tree->key_) {
		return typename Node<Key,Value>::Tree(new Node<Key, Value>(tree->left_, key, value, tree->right_, tree->height_));
	}
	else if( key < tree->key_) {
		return bal(add(key, value, tree->left_), tree->key_, tree->value_, tree->right_);
	}
	else {
		return bal(tree->left_, tree->key_, tree->value_, add(key, value, tree->right_));
	}
}

template <class Key, class Value>
Value find(const typename boost::shared_ptr<Node<Key,Value> >& tree, const Key& key){
  if(tree->key_ == key)
  	return tree->value_;
  if(key < tree->key_)
  	return find(tree->left_, key);
  else
  	return find(tree->right_,key);
}

}
