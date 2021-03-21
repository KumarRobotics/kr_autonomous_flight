/**
 * @file control.h
 * @brief Control classes
 */
#ifndef MPL_CONTROL_H
#define MPL_CONTROL_H

namespace Control {
/// Enum for control input
enum Control {
  NONE = 0,           ///< default uninitialized value
  VEL = 0b00001,      ///< control input is vel
  ACC = 0b00011,      ///< control input is acc
  JRK = 0b00111,      ///< control input is jrk
  SNP = 0b01111,      ///< control input is snp
  VELxYAW = 0b10001,  ///< control input is vel and yaw
  ACCxYAW = 0b10011,  ///< control input is acc and yaw
  JRKxYAW = 0b10111,  ///< control input is jrk and yaw
  SNPxYAW = 0b11111   ///< control input is snp and yaw
};
}  // namespace Control
#endif
