/*
 * Created By: Logan Fillo
 * Created On: October 19 2019
 * Description: thruster command interface
 */

#ifndef HARDWARE_INTERFACE_THRUSTER_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_THRUSTER_COMMAND_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface
{

/**
 * @class hardware_interface::ThrusterHandle
 * 
 * Handle used for sharing memory between thruster command interface and controller
 */
class ThrusterHandle 
{
public:
  ThrusterHandle () : cmd_(0) {}

  /**
   * @param name The thruster's name
   * @param cmd A pointer to the storage for this thrusters output command
   */
  ThrusterHandle (const std::string& name, double* cmd)
    : cmd_(cmd), name_ (name)
  {
    if (!cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Command data pointer is null.");
    }
  }

  std::string getName() const {return name_;}
  void setCommand(double command) {assert(cmd_); *cmd_ = command;}
  double getCommand() const {assert(cmd_); return *cmd_;}

  double* getCommandPtr() {return cmd_;}

private:
  std::string name_;
  double* cmd_;

};

/**
 * @class hardware_interface::ThrusterCommandInterface
 * 
 * A hardware interface for commanding a thruster
 */
class ThrusterCommandInterface : public HardwareResourceManager<ThrusterHandle> {};

}
#endif // HARDWARE_INTERFACE_THRUSTER_COMMAND_INTERFACE_H