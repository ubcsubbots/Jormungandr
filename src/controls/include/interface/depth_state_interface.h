#ifndef HARDWARE_INTERFACE_DEPTH_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_DEPTH_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface
{

/**
 * @class hardware_interface::DepthStateHandle
 * 
 * Handle used for sharing memory between depth state interface and controller
 */
class DepthStateHandle 
{
public:
  DepthStateHandle () : state_(0) {}

  /**
   * @param name Identifer of sensor type
   * @param state A pointer to the storage for depth's state
   */
  DepthStateHandle (const std::string& name, double* state)
    : state_(state), name_ (name)
  {
    if (!state_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. State data pointer is null.");
    }
  }

  std::string getName() const {return name_;}
  double getState() const {assert(state_); return *state_;}

  double* getStatePtr() {return state_;}

private:
  std::string name_;
  double* state_;
};

/**
 * @class hardware_interface::DepthStateInterface
 * 
 * A hardware interface for reading depth state
 */
class DepthStateInterface : public HardwareResourceManager<DepthStateHandle> {};

}

#endif //HARDWARE_INTERFACE_DEPTH_STATE_INTERFACE_H