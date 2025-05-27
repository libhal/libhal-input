// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>

#include <libhal/i2c.hpp>

namespace hal::input {
/**
 * @brief Driver for nunchuck i2c gamepad
 *
 */
class nunchuck
{
public:
  /**
   * @brief Holds information about gamepad state, and contains functions to
   * return translated data.
   *
   */
  struct gamepad_data
  {
    /// @brief byte array containing data about gamepad state.
    std::array<hal::byte, 6> data;
    /**
     * @brief Get x-axis data of the joystick.
     * @return uint8_t representing the x-axis data of the joystick. Center
     * position is 128. 0-127 indicates the stick is pointed left and 129-255
     * indicates the stick is pointed right.
     */
    std::uint8_t joystick_x();
    /**
     * @brief Get y-axis data of the joystick.
     * @return uint8_t representing the y-axis data of the joystick. Center
     * position is 128. 0-127 indicates the stick is pointed dwon and 129-255
     * indicates the stick is pointed up.
     */
    std::uint8_t joystick_y();
    /**
     * @brief Get x-axis data of the accelerometer sensor.
     * @return uint16_t representing the x-axis data of the accelerometer.
     * Center position is about 512. Reducing values means left movement,
     * increasing values means right movement.
     */
    std::uint16_t accelerometer_x();
    /**
     * @brief Get y-axis data of the accelerometer sensor.
     * @return uint16_t representing the y-axis data of the accelerometer.
     * Center position is about 512. Reducing values means backward movement,
     * increasing values means forward movement.
     */
    std::uint16_t accelerometer_y();
    /**
     * @brief Get z-axis data of the accelerometer sensor.
     * @return uint16_t representing the z-axis data of the accelerometer.
     * Center position is about 512. Reducing values means downward movement,
     * increasing values means upward movement.
     */
    std::uint16_t accelerometer_z();
    /**
     * @brief Get z button pressed state.
     * @return true representing if the z button is pressed
     * @return false representing if the z button is unpressed
     */
    bool z_button();
    /**
     * @brief Get c button pressed state.
     * @return true representing if the c button is pressed
     * @return false representing if the c button is unpressed
     */
    bool c_button();
  };

  /**
   * @brief Create a nunchuck object
   * @param p_i2c - the driver for the i2c bus the nunchuck is connected to
   */
  nunchuck(hal::i2c& p_i2c);
  gamepad_data read();

private:
  hal::i2c* m_i2c;
};
}  // namespace hal::input
