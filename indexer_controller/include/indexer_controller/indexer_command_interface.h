#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <indexer_controller/indexer_state_interface.h>

namespace hardware_interface {

class IndexerCommandHandle : public IndexerStateHandle {
 public:
  IndexerCommandHandle() : cmd_(nullptr) {}

  IndexerCommandHandle(const IndexerStateHandle& state_handle, float* cmd_)
      : IndexerStateHandle(state_handle), cmd_(cmd_) {
    if (cmd_ == nullptr) {
      throw HardwareInterfaceException("Cannot create indexer command handle, " + getName() + ". cmd_ pointer is null");
    }
  }

  void setCommand(float cmd) {
    assert(cmd_);
    *cmd_ = cmd;
  }

  float getCommand() {
    assert(cmd_);
    return *cmd_;
  }

 private:
  float* cmd_{nullptr};
};

class IndexerCommandInterface : public HardwareResourceManager<IndexerCommandHandle, ClaimResources> {};

}  // namespace hardware_interface
