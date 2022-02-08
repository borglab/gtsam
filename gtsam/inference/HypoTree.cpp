#include <gtsam/inference/HypoTree.h>

namespace gtsam {

  HypoLayer& HypoLayer::toPrevLayer() {
    return belong_tree_ptr_->getLayer(layer_idx_ - 1);
  }

  HypoLayer& HypoLayer::toDiffLayer(const int& layer_diff) {
    return belong_tree_ptr_->getLayer(layer_idx_ + layer_diff);
  }

  //[MH-C]: TEST
  void HypoLayer::printAllDim() {

    const size_t& common_dim = belong_tree_ptr_->common_dim_;
        
    std::cout << "printAllDim(): " << std::endl;
    std::cout << "common_dim: " << common_dim << std::endl;
    for (HypoListCstIter it = node_list_.begin(); it != node_list_.end(); ++it) {
      const size_t acc_dim = (*it)->accumulated_dim_;
      std::cout << acc_dim << " " << getChi2(acc_dim + common_dim) << std::endl;
    }
    std::cout << std::endl;
  }
  
  void HypoLayer::setKeyMap (const Key& key) {
    belong_tree_ptr_->addKeyLayerMap(key, this);
  }
  
  void HypoLayer::removeKeyMap (const Key& key) {
    belong_tree_ptr_->eraseKeyLayerMap(key);
  }

  //[MH-A]: recursive find num of descendant at a given layer
  size_t HypoNode::findDescendantNum(const int& layer_diff) {

    if (is_pruned_) {
      return 0;
    }
    // layer_diff MUST be >= 0
    if (layer_diff == 0) { //arrive the desired layer
      return 1;
    
    } else if (layer_diff > 0) {
      const int this_layer_idx = belong_layer_ptr_->getLayerIdx();
      const int des_layer_idx = this_layer_idx + layer_diff;
      HypoList& des_node_list = belong_layer_ptr_->getBelongTreePtr()->layer_arr_[des_layer_idx]->getNodeList();

      size_t sum_child_num = 0;  
      for (HypoListCstIter it = des_node_list.begin(); it != des_node_list.end(); ++it) {
        HypoNode* anc_node_ptr = (*it)->findAncestor(this_layer_idx);
        if (anc_node_ptr == (this)) { //link back to itself
          sum_child_num++;
        }
      }
      return sum_child_num;

    } else { // < 0
      std::cout << "ERROR: findDescendantNum() should NEVER have input layer_diff < 0 !!" << std::endl; 
      return 0;
    }
  } // END findDescendantNum()

} // END gtsam namespace
