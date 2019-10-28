/*
 * Created By: Logan Fillo
 * Created On: October 19 2019
 * Description: Thruster array interface
 */

#ifndef HARDWARE_INTERFACE_THRUSTER_ARRAY_INTERFACE_H
#define HARDWARE_INTERFACE_THRUSTER_ARRAY_INTERFACE_H

#include <types/data_types.h>
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
        thruster_array_(0),
        imu_sensor_(0),
        depth_sensor_(0)
  {}

  /**
   * Constructs a thruster array handle
   * 
   * @param name The thruster's name
   * @param thruster_array A pointer to the location of thruster array data
   * @param imu_sensor A pointer to the location of imu sensor data
   * @param depth_sensor A pointer to the location of depth sensor data
   */
  ThrusterArrayHandle (
    const std::string& name,
    ThrusterArrayData* thruster_array,
    ImuSensorData* imu_sensor,
    DepthSensorData* depth_sensor
  )
    : name_(name),
      thruster_array_(thruster_array),
      imu_sensor_(imu_sensor),
      depth_sensor_(depth_sensor)
  {}

  std::string getName() const {return name_;}
  ThrusterArrayData* getThrusterArray() const {return thruster_array_;}
  ImuSensorData* getImuSensor() const {return imu_sensor_;}
  DepthSensorData* getDepthSensor() const {return depth_sensor_;}

  void commandThrusterArray(ThrusterArrayData thruster_array_cmd) const {
    assert(thruster_array_);
    *thruster_array_ = thruster_array_cmd;
  }

  void setImuSensorData(ImuSensorData imu_sensor_data) const {
    assert(imu_sensor_);
    *imu_sensor_ = imu_sensor_data;
  }

  void setDepthSensorData(DepthSensorData depth_sensor_data) const {
    assert(depth_sensor_);
    *depth_sensor_ = depth_sensor_data;
  }

private:
  std::string name_;
  ThrusterArrayData* thruster_array_;
  ImuSensorData* imu_sensor_;
  DepthSensorData* depth_sensor_;
};

/**
 * @class hardware_interface::ThrusterArrayInterface
 * 
 * A hardware interface for commanding a thruster array
 */
class ThrusterArrayInterface : public HardwareResourceManager<ThrusterArrayHandle> {};

}
#endif // HARDWARE_INTERFACE_THRUSTER_ARRAY_INTERFACE_H