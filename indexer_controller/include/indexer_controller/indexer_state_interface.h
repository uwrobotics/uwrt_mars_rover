#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace hardware_interface {

class IndexerStateHandle {
 public:
  IndexerStateHandle() : raw_pos_(nullptr) {}
  IndexerStateHandle(std::string name, const float* raw_pos) : name_(std::move(name)), raw_pos_(raw_pos) {
    if (raw_pos_ == nullptr) {
      throw HardwareInterfaceException("Cannot create indexer state handle, " + name_ + ". raw_pos_ pointer is null");
    }
  }

  std::string getName() const {
    return name_;
  }

  float getRawPos() const {
    assert(raw_pos_);
    return *raw_pos_;
  }

 private:
  std::string name_;
  const float* raw_pos_{nullptr};
};

class IndexerStateInterface : public HardwareResourceManager<IndexerStateHandle> {};

}  // namespace hardware_interface