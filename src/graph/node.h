// Author: Tucker Haydon

#ifndef PATH_PLANNING_GRAPH_NODE_H
#define PATH_PLANNING_GRAPH_NODE_H

#include <string>
#include <cstdlib>
#include <cstring>
#include <iostream>

namespace path_planning {
  class Node {
    private:
      uint8_t* data_;
      size_t data_size_;
      bool heap_allocated_{false};

      void CleanUp();

    public:
      Node() {}
      ~Node();

      // Copy constructor and assignment
      Node(const Node& other);
      Node& operator=(const Node& other);

      // Prevent moves (for now)
      Node& operator=(Node&& other) noexcept = delete;
      Node(Node&& other) noexcept  = delete;
      
      const uint8_t* Data() const;
      bool SetData(const uint8_t* data, const size_t size);
  
      bool operator==(const Node& other) const;

		  struct Hash {
        size_t operator()(const Node& node) const;
		  };
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline Node::Node(const Node& other) {
    this->SetData(other.data_, other.data_size_);
  }

  inline Node& Node::operator=(const Node& other) {
    this->SetData(other.data_, other.data_size_);
  }

  inline Node::~Node() {
    this->CleanUp();
  }

  inline void Node::CleanUp() {
    if(true == this->heap_allocated_) {
      std::free(this->data_);
      this->heap_allocated_ = false;
    }
  }

  inline const uint8_t* Node::Data() const {
    return this->data_;
  }

  inline bool Node::SetData(const uint8_t* data, const size_t size) {
    this->CleanUp();
    this->data_size_ = size;
    this->data_ = reinterpret_cast<uint8_t*>(std::malloc(this->data_size_));
    this->heap_allocated_ = true;
    std::memcpy(this->data_, data, this->data_size_);
    return true;
  }
  
  inline bool Node::operator==(const Node& other) const {
    return 
      this->data_size_ == other.data_size_ &&
      0 == std::memcmp(this->data_, other.data_, this->data_size_);
  }

  // Reference: 
  // https://stackoverflow.com/questions/20511347/a-good-hash-function-for-a-vector
  inline size_t Node::Hash::operator()(const Node& node) const {
    size_t seed = node.data_size_;
    for(size_t idx = 0; idx < node.data_size_; ++idx) {
      seed ^= node.data_[idx] + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
}

#endif
