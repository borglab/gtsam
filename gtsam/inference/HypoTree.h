#pragma once

#include <boost/math/distributions/chi_squared.hpp> //for getChi2()

#include <utility> //std::pair

#include <stdbool.h>

#include <boost/ptr_container/serialize_ptr_map.hpp>

#include <iostream>

#include <gtsam/base/Vector.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/ConcurrentMap.h>

#include <gtsam/inference/Key.h>

#include <gtsam/inference/Factor.h>

namespace gtsam {

  class HypoTree;
  class HypoLayer;
  class HypoNode;

  struct MHParams {
      size_t desiredHypoTH; 
      size_t limitHypoTH; 
     
      bool isStrictTH;
      bool isPrintPruningDetails;
      
      MHParams(
        size_t _desired_hypo_th = 10, //10 
        size_t _limit_hypo_th = 20, //2 times? 
        bool is_strict_th = false,
        bool is_print_pruning_details = false
      ) : desiredHypoTH(_desired_hypo_th), limitHypoTH(_limit_hypo_th), isStrictTH(is_strict_th), isPrintPruningDetails(is_print_pruning_details) {
        if (desiredHypoTH < 1) {
          std::cout << "Should NOT set desiredHypoTH = " << desiredHypoTH << " (MUST >= 1)" << std::endl;
        }
        if (limitHypoTH < desiredHypoTH) {
          std::cout << "Should NOT set limitHypoTH < desiredHypoTH (at least =)" << std::endl;
        }
      }
  }; //struct MHParams

  class PruningRecord {
    public:
      size_t original_size_;
      std::vector<size_t> pruned_idx_arr_;
  };

  class GTSAM_EXPORT HypoNode {
    public:
      typedef boost::shared_ptr<Value> sharedImplyGeneric; //same as sharedImplyGeneric ...
      typedef boost::shared_ptr<Vector> sharedVector; //same as sharedImplyGeneric ...

      typedef FastMap<Key, sharedImplyGeneric> KeyValueMap; //NOT ConcurrentMap<>
      typedef typename KeyValueMap::iterator KVMapIter;
      typedef typename KeyValueMap::const_iterator KVMapCstIter;

      typedef std::vector<HypoNode*> HypoArr;
      
      typedef std::list<HypoNode*> HypoList;
      typedef typename HypoList::iterator HypoListIter;
      typedef typename HypoList::const_iterator HypoListCstIter;

      const HypoLayer* belong_layer_ptr_;
      HypoList child_list_;
      
      HypoArr ancestor_arr_;
      size_t ancestor_level_; //200 layers per level
      HypoNode* prev_level_ptr_;
  
      KeyValueMap key_value_map_;

      const int mode_id_; //corresponds to each mode in MN-sourse factor
      bool is_pruned_; //stop update from this node
      
      //[MH-C]: 
      int detach_count_; //TODO: now recorded but useless...
      size_t accumulated_dim_; //used for chi2 test...
    
      HypoNode() : belong_layer_ptr_(NULL), ancestor_level_(0), mode_id_(-1), is_pruned_(false), detach_count_(0), accumulated_dim_(0) {
        std::cout << "HypoNode() constructor without input should NEVER be called" << std::endl;
      }
      HypoNode(HypoLayer* belong, const int& mode) : belong_layer_ptr_(belong), ancestor_level_(0), mode_id_(mode), is_pruned_(false), detach_count_(0), accumulated_dim_(0) {
        // Setup a HypoNode
      }
      HypoNode(HypoLayer* belong, const int& mode, HypoNode* parent) : belong_layer_ptr_(belong), mode_id_(mode), is_pruned_(false), detach_count_(0), accumulated_dim_(0) {
        // Setup a HypoNode with prev_child and this_ancestor
        parent->child_list_.push_back(this);
        
        if (parent->ancestor_arr_.size() == (200 - 1)) {
          prev_level_ptr_ = parent;
          ancestor_arr_.resize(0);
          ancestor_level_ = parent->ancestor_level_ + 1;
        } else{
          prev_level_ptr_ = parent->prev_level_ptr_;
          ancestor_arr_.resize(parent->ancestor_arr_.size() + 1);
          std::copy(parent->ancestor_arr_.begin(), parent->ancestor_arr_.end(), ancestor_arr_.begin());
          ancestor_arr_.back() = parent;
          ancestor_level_ = parent->ancestor_level_;
        }

        //[MH-E]:
        detach_count_ = parent->detach_count_;
        accumulated_dim_ = parent->accumulated_dim_;
      }

      const int getModeId() {
        return mode_id_;
      }

      HypoNode* getParent() { 
        if (ancestor_arr_.size() == 0) {
          return prev_level_ptr_;
        } else {
          return ancestor_arr_.back();
        }
      }
      
      HypoNode* findAncestor(const size_t& idx) { //idx is a given layer_idx_
        // Use ancestor_arr_ to get corresponding node
        //TODO: Might have to rewrite a version without ancestor_arr_ for memory efficiency
        const size_t level = idx / 200; 
        if (level == ancestor_level_) {
          const size_t lv_idx = (idx % 200);
          if (lv_idx < ancestor_arr_.size()) {
            return ancestor_arr_[lv_idx];
          } else if (lv_idx == ancestor_arr_.size()) {
            return (this);
          } else { // go to previous level
            std::cout << "MH-ERROR: findAncestor(idx): idx out of range (" << lv_idx << "/" << ancestor_arr_.size() << ")" << std::endl;
            return NULL;
          }
        } else if (level < ancestor_level_) {
          // Go to prev_level
          return prev_level_ptr_->findAncestor(idx);
        } else {
          std::cout << "MH-ERROR: findAncestor(idx): level out of range" << std::endl;
          std::cout << idx << std::endl;
          std::cout << level << " " << ancestor_level_ << std::endl;
          return NULL;
        }
      }
      
      //[MH-A]: recursive find num of descendant at a given layer
      size_t findDescendantNum(const int& layer_diff);


      //[MH-A]: recursive find num of descendant at the given last layer
      
      //[MH-S]:
      void findSegDescendantNum(std::list<size_t>& out_num_list, const int& layer_diff, const int& layer_seg) { //layer_seg MUST >= 1, represents how many layers before "this_layer + layer_diff"
       
        if (layer_diff == 0) {
          std::cout << "ERROR: layer_diff == 0 in findSegDescendantNum()" << std::endl;
          return;
        }
       
        if (is_pruned_) {
          return;
        }
        
        if (layer_diff > layer_seg) { //arrive the desired layer
          for (HypoListCstIter it = child_list_.begin(); it != child_list_.end(); ++it) {
            
            (*it)->findSegDescendantNum(out_num_list, (layer_diff - 1), layer_seg);
            
          }
        } else if (layer_diff == layer_seg) {
          size_t sum_child_num = 0; 
 
          for (HypoListCstIter it = child_list_.begin(); it != child_list_.end(); ++it) {
            //[MH-A]: goes to another recursive
            sum_child_num += ((*it)->findDescendantNum(layer_diff - 1));
          }

          out_num_list.push_back(sum_child_num);

        } else { //layer_diff < layer_seg
          std::cout << "ERROR: layer_diff < layer_seg in findSegDescendantNum()" << std::endl;
        }
        
      } // END findSegDescendantNum()
      
      //[MH-A]: add
      void addKeyValuePair(const Key j, const sharedImplyGeneric& value) {
        key_value_map_.insert(std::make_pair(j, value));
      }
     
      //[MH-A]: remove
      void removeValueLink(const Key& j) {
        key_value_map_.erase(key_value_map_.find(j));
      }
      
      void flagToPrune() {
        if (is_pruned_) {
          //WARNING: This HypoNode is already pruned
          //TODO: Do nothing for now...
        }
        is_pruned_ = true;
      }

      //[MH-C]: MUST prune
      void pruneThisNode() {
        //TODO: Do nothing for now. In current setup flagToPrune() completes pruning the node already...
      } // END pruneThisNode()
      
      //[MH-C]: check if all children are pruned already
      bool isPrunable() {
        for (HypoListCstIter it = child_list_.begin(); it != child_list_.end(); ++it) {
          if ( !((*it)->is_pruned_) ) { //at least one still valid
            return false;
          }
        }
        return true;
      }

      ~HypoNode() {}
  }; //END HypoNode

  class GTSAM_EXPORT HypoLayer {
    public:
      typedef boost::shared_ptr<Factor> sharedFactor;

      typedef std::list<HypoNode*> HypoList;
      
      typedef std::list<Key> KeyList;  
     
      typedef typename HypoList::iterator HypoListIter;
      typedef typename HypoList::const_iterator HypoListCstIter;

      typedef std::vector<PruningRecord*> RecordArr;
   
    private:   
      const int layer_idx_;
      
      HypoTree* belong_tree_ptr_;
      
      HypoList node_list_;
      
      //[MH-C]:
      RecordArr record_arr_;
      HypoList pruned_list_; //pruned-nodes are moved to here?
    
      sharedFactor source_factor_; //input raw factor that causes this hypo split... Must be MH, usually nonlinear
      //FactorList target_factor_list_; //resulting linearized factors... All must be MH (since we only need to search from Factor to Value/Vector...)
      KeyList key_list_; //each Key to one resulting MHValue/MHVector
    
    public:
      HypoLayer() : layer_idx_(-1) {//should never be used
        std::cout << "HypoLayer() without input arg should never be used" << std::endl;
      }

      HypoLayer(const int& idx, HypoTree* belong, const int& all_mode_size) : layer_idx_(idx), belong_tree_ptr_(belong) {
        // Create a new layer with corresponding number of nodes
        if (idx == 0) {
          node_list_.push_back(new HypoNode(this, 0) );
        } else {
          HypoList& prev_node_list = toPrevLayer().getNodeList();
          for (HypoListIter nit = prev_node_list.begin(); nit != prev_node_list.end(); ++nit) {
            //[MH-C]: careful
            if ( !((*nit)->is_pruned_) ) {
              for (int i = 0; i < all_mode_size; ++i) {
                node_list_.push_back(new HypoNode(this, i, (*nit)) );
              }
            }
          } // END for (nit : prev_node_list)
        } // END if-else
      }
      ~HypoLayer() {}

      //[MH-E]:
      void updateDimWithDetachable(const bool& set_detach, const size_t& dim, const int& all_mode_size = 0) {
        if (set_detach) {
          int count_hypo = 1; //NOTICE: trick to always flag at the last of the mode_size 
          for (HypoListIter it = node_list_.begin(); it != node_list_.end(); ++it) {
            if ( (count_hypo%all_mode_size) == 0 ) {
              (*it)->detach_count_++;
            } else {
              (*it)->accumulated_dim_ += dim;
            }
            count_hypo++;
          } 
        
        } else {
          for (HypoListIter it = node_list_.begin(); it != node_list_.end(); ++it) {
            (*it)->accumulated_dim_ += dim;
          } 
        }

      } // END updateDimWithDetachable()
  
      //[MH-C]:
      static double getChi2(const size_t& dof, const double& p_value = 0.95) {
        if (dof == 0) {
          //WARNING: getChi2() input dof = 0, return Inf 
          return std::numeric_limits<double>::infinity();
        } else {
          return boost::math::quantile(boost::math::chi_squared(dof), p_value);
        }
      }
      //[MH-C]: TEST
      void printAllDim();
    
      void setSource(const sharedFactor& factor) {
        source_factor_ = factor;
      }

      const int& getLayerIdx() const {
        return layer_idx_;
      }

      HypoTree* getBelongTreePtr() const {
        return belong_tree_ptr_;
      }

      size_t getNodeSize() const {
        return node_list_.size();
      }

      HypoList& getNodeList() {
        return node_list_;
      }
      
      RecordArr& getRecordArr() {
        return record_arr_;
      }

      bool isModeIdExist(const int& mode_id) {
        for (HypoListIter it = node_list_.begin(); it != node_list_.end(); ++it) {
          if ( (*it)->getModeId() == mode_id ) {
            return true;
          }
        }
        return false;
      }

      bool isModeConverged() {
        const int mode_id = (node_list_.front())->getModeId();
        for (HypoListIter it = node_list_.begin(); it != node_list_.end(); ++it) {
          if ( (*it)->getModeId() != mode_id ) {
            return false;
          }
        }
        return true;
      }

      HypoLayer& toPrevLayer();
      
      void setKeyMap(const Key& key);
      void removeKeyMap(const Key& key);
      
      HypoLayer& toDiffLayer(const int& layer_diff);

      //[MH-C]:
      void pruneFlagedNodes() {

        PruningRecord* record_ptr = new PruningRecord();
        record_ptr->original_size_ = getNodeSize();
        std::vector<size_t>& pruned_idx_arr = record_ptr->pruned_idx_arr_;
        size_t pruned_idx = 0;
        for (HypoListIter it = node_list_.begin(); it != node_list_.end(); ++it) {
          if ((*it)->is_pruned_) {
            //recursive checking all children of its parent (per layer)
            (*it)->pruneThisNode();
            if ( ((*it)->getParent())->isPrunable() ) {
              // Prune parent
              ((*it)->getParent())->flagToPrune(); 
            }
            pruned_list_.push_back(*it);
            it = node_list_.erase(it);
            --it;
            
            pruned_idx_arr.push_back(pruned_idx);
          }
          ++pruned_idx;
        }
        
        if (pruned_idx_arr.size() == 0) {
          // Recover: nothing should happen
          delete record_ptr;
        } else {
          record_arr_.push_back(record_ptr);
          // Recursively try to prune each HypoLayer
          toPrevLayer().pruneFlagedNodes();
        }
      }

  }; //END HypoLayer

  class GTSAM_EXPORT HypoTree {
    public:
      typedef boost::shared_ptr<Factor> sharedFactor;
      
      typedef std::vector<HypoLayer*> LayerArr;
      typedef FastMap<Key, HypoLayer*> KeyLayerMap;
    
      LayerArr layer_arr_;
      KeyLayerMap key_layer_map_; //might be useless
      
      //[MH-C]:
      size_t common_dim_;
    
      HypoTree() : common_dim_(0) { //the ONLY constructor
        // Create root (single-hypo at the very beginning)    
        layer_arr_.push_back(new HypoLayer(0, this, 1));
      }
      ~HypoTree() {}
      
      HypoNode* root() {
        return layer_arr_.front()->getNodeList().front();
      }

      void addLayer(const int& mode_size, const size_t& dim, const bool& is_loop, const bool& is_detachable) {
        // Add a new layer in the current HypoTree
        const int idx = layer_arr_.size();
        if (is_detachable) {
          //[MH-E]:
          layer_arr_.push_back(new HypoLayer(idx, this, (mode_size + 1)));
        } else {
          layer_arr_.push_back(new HypoLayer(idx, this, mode_size));
        }
        if (is_loop) {
          if (is_detachable) {
            //[MH-E]:
            layer_arr_.back()->updateDimWithDetachable(true, dim, mode_size + 1);
          } else {
            layer_arr_.back()->updateDimWithDetachable(false, dim);
          }
        } // END if-is_loop
      }

      //[MH-C]:
      void updateCommonDim (const size_t& dim) {
        common_dim_ += dim;
      }
      
      HypoLayer& getLayer(const int& idx) {
        return (*(layer_arr_[idx]));
      }
      
      HypoLayer& getLastLayer() {
        return (*(layer_arr_.back()));
      }

      void addKeyLayerMap(const Key& j, HypoLayer* hypo_layer_ptr) {
        // Associate a key with a layer
        key_layer_map_.insert(std::make_pair(j, hypo_layer_ptr));
      }
      
      void eraseKeyLayerMap(const Key& j) {
        // Cancel the association between a key and a layer
        key_layer_map_.erase(key_layer_map_.find(j));
      }

      HypoLayer* setLatestLayerSource(const sharedFactor& factor) {
        // Input is always a descendant of MHNoiseModelFactor
        HypoLayer* latest_layer = layer_arr_.back();
        latest_layer->setSource(factor);
        return latest_layer;
      }

      int calculateOriginalHypoNum() {
        int total_mode_num = 1;
        HypoNode* node = *(layer_arr_.back()->getNodeList().begin());
        for (size_t a = 0; a < (200*node->ancestor_level_ + (node->ancestor_arr_.size())); ++a) {
          const int mode_num = node->findAncestor(a)->child_list_.size();
          total_mode_num *= mode_num;
        }
        return total_mode_num;
      }

  }; //END HypoTree

} //namespace gtsam
