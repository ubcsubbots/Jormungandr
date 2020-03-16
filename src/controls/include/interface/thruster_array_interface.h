/*
 * Created By: Logan Fillo
 * Created On: October 19 2019
 * Description: Thruster array interface
 */

#ifndef HARDWARE_INTERFACE_THRUSTER_ARRAY_INTERFACE_H
#define HARDWARE_INTERFACE_THRUSTER_ARRAY_INTERFACE_H

#include <AUVState.h>
#include <ThrusterArrayCommand.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

/**
 * Struct to store handle data 
 */

namespace hardware_interface
{

/**
 * @class hardware_interface::ThrusterArrayHandle
 * 
 * Handle used for sharing memory between hardware interface and thruster array controller
 */
class ThrusterArrayHandle 
{
public:

  ThrusterArrayHandle()
      : name_(),
        auv_state_(0),
        thruster_array_(0)
  {}

  /**
   * Constructs a thruster array handle
   * 
   * @param name The thruster's name
   * @param auv_state A pointer to the location of AUV state
   * @param thruster_array A pointer to the location of thruster array command
   */
  ThrusterArrayHandle (
    const std::string& name,
    AUVState* auv_state,
    ThrusterArrayCommand* thruster_array
  )
    : name_(name),
      auv_state_(auv_state),
      thruster_array_(thruster_array)
  {}

  std::string getName() const {return name_;}
  ThrusterArrayCommand* getThrusterArray() const {return thruster_array_;}
  AUVState* getAUVState() const {return auv_state_;}

  void commandThrusterArray(ThrusterArrayCommand thruster_array_cmd) const {
    assert(thruster_array_);
    *thruster_array_ = thruster_array_cmd;
  }

private:
  std::string name_;
  AUVState* auv_state_;
  ThrusterArrayCommand* thruster_array_;
};

/**
 * @class hardware_interface::ThrusterArrayInterface
 * 
 * A hardware interface for commanding a thruster array
 */
class ThrusterArrayInterface : public HardwareResourceManager<ThrusterArrayHandle> {};

}
#endif // HARDWARE_INTERFACE_THRUSTER_ARRAY_INTERFACE_H