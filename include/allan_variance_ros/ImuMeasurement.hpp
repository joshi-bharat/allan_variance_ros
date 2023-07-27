#pragma once

#include <Eigen/Core>
#include <string>

///
/// \brief Contains data from the IMU mesaurements.
///
class ImuMeasurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  uint64_t t{};            ///< ROS time message received (nanoseconds).

  Eigen::Vector3d I_a_WI;  ///< Raw acceleration from the IMU (m/s/s)
  Eigen::Vector3d I_w_WI;  ///< Raw angular velocity from the IMU (deg/s)

  ~ImuMeasurement() {}
  ImuMeasurement();
  ImuMeasurement(const uint64_t _t, const Eigen::Vector3d& _I_a_WI, const Eigen::Vector3d& _I_w_WI);
  friend std::ostream& operator<<(std::ostream& stream, const ImuMeasurement& meas);
};

///
/// \brief Contains data from the magnetometer measurements.
///

class MagneticMeasurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  uint64_t t{};            ///< ROS time message received (nanoseconds).
  Eigen::Vector3d I_m_WI;  ///< Raw magnetic field from the magnetometer (uT)

  ~MagneticMeasurement() {}
  MagneticMeasurement();
  MagneticMeasurement(const uint64_t _t, const Eigen::Vector3d& _I_m_WI);

  friend std::ostream& operator<<(std::ostream& stream, const MagneticMeasurement& meas);
};
