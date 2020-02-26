#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace hardware_interface {

class IndexerStateHandle {
public:
  // TODO (wraftus) maybe add max index to the IndexState
  struct IndexerState {
    uint16_t cur_index;
    float raw_pos;
    uint16_t to_index;
    uint16_t from_index;
  };

  IndexerStateHandle()
    : name_(), 
      cur_index_(nullptr), 
      raw_pos_(nullptr), 
      to_index_(nullptr),
      from_index_(nullptr) {}

  IndexerStateHandle(const std::string& name, const IndexerState* state)
    : name_(name), 
      cur_index_(&state->cur_index),
      raw_pos_(&state->raw_pos),
      to_index_(&state->to_index),
      from_index_(&state->from_index) {}

  IndexerStateHandle(const std::string& name, const uint16_t* cur_index, const float* raw_pos, const uint16_t* to_index, const uint16_t* from_index)
    : name_(name),
      cur_index_(cur_index),
      raw_pos_(raw_pos),
      to_index_(to_index),
      from_index_(from_index) {

    if(!cur_index_)
      throw HardwareInterfaceException("Cannot create indexer state handle, " + name_ + ". cur_index pointer is null");
    if(!raw_pos_)
      throw HardwareInterfaceException("Cannot create indexer state handle, " + name_ + ". raw_pos_ pointer is null");
    if(!to_index_)
      throw HardwareInterfaceException("Cannot create indexer state handle, " + name_ + ". to_index_ pointer is null");
    if(!from_index_)
      throw HardwareInterfaceException("Cannot create indexer state handle, " + name_ + ". from_index_ pointer is null");
  }  

  std::string getName() const { return name_; }

  uint16_t getCurIndex() const {
    assert(cur_index_);
    return *cur_index_;
  }
  float getRawPos() const {
    assert(raw_pos_);
    return *raw_pos_;
  }
  uint16_t getToIndex() const {
    assert(to_index_);
    return *to_index_;
  }
  uint16_t getFromIndex() const {
    assert(from_index_);
    return *from_index_;
  }
    
private:
  std::string name_;
  const uint16_t* cur_index_;
  const float* raw_pos_;
  const uint16_t* to_index_;
  const uint16_t* from_index_;
};

class IndexerStateInterface : public HardwareResourceManager<IndexerStateHandle> {};

} // namespace hardware_interface