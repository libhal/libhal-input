// Copyright 2024 Khalil Estell
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

#include <libhal-input/gamepad/nunchuck.hpp>
#include <libhal-util/i2c.hpp>

namespace hal::input {
nunchuck::nunchuck(hal::i2c& p_i2c)
  : m_i2c(&p_i2c)
{
  // handshake byte (0x40) is needed to initialize
  hal::write(
    *m_i2c, 0x52, std::array<hal::byte, 1>{ 0x40 }, hal::never_timeout());
}

nunchuck::gamepad_data nunchuck::read()
{
  gamepad_data controller{};
  hal::write_then_read(*m_i2c,
                       0x52,
                       std::array<hal::byte, 1>{ 0x00 },
                       controller.data,
                       hal::never_timeout());
  return controller;
}

std::uint8_t nunchuck::gamepad_data::joystick_x()
{
  return data[0];
}

std::uint8_t nunchuck::gamepad_data::joystick_y()
{
  return data[1];
}

std::uint16_t nunchuck::gamepad_data::accelerometer_x()
{
  // Accelerometer data is 10-bits split up with upper bits having a whole byte
  // to themselves and the lower 2 bits are contained in data[5].

  // x-axis upper bits are in data[2]
  // Put the upper bits into a 16-bit int and left shift over 2 to make room for
  // lower bits.
  std::uint16_t x_upper_bits = data[2] << 2;
  // x-axis lower bits are bit 2 and 3 of data[5].
  // Used a bit mask with & operator to get only these 2 bits, then shift them
  // all the way to the right to be combined with upper bits.
  std::uint16_t x_lower_bits = (data[5] & 0b0000000000001100) >> 2;
  // Combine upper and lower bits by using an | operator
  return x_upper_bits | x_lower_bits;
}

std::uint16_t nunchuck::gamepad_data::accelerometer_y()
{
  // y-axis upper bits are in data[3]
  std::uint16_t y_upper_bits = data[3] << 2;
  // y-axis lower bits are bit 4 and 5 of data[5].
  std::uint16_t y_lower_bits = (data[5] & 0b0000000000110000) >> 4;
  return y_upper_bits | y_lower_bits;
}

std::uint16_t nunchuck::gamepad_data::accelerometer_z()
{
  // z-axis upper bits are in data[4]
  std::uint16_t z_upper_bits = data[4] << 2;
  // z-axis lower bits are bit 6 and 7 of data[5].
  std::uint16_t z_lower_bits = (data[5] & 0b0000000011000000) >> 6;
  return z_upper_bits | z_lower_bits;
}

bool nunchuck::gamepad_data::z_button()
{
  // z button state is bit 0 of data[5].
  // Originally returns 0 when pressed, inverted to return true when pressed.
  return !(data[5] & 0b00000001);
}

bool nunchuck::gamepad_data::c_button()
{
  // c button state is bit 1 of data[5].
  return !(data[5] & 0b0000010);
}
}  // namespace hal::input
