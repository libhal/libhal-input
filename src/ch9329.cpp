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

#include <libhal-input/ch9329.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/serial.hpp>
#include <libhal/serial.hpp>
#include <libhal/units.hpp>

namespace hal::input {

constexpr hal::byte header_byte_1 = 0x57;
constexpr hal::byte header_byte_2 = 0xAB;
constexpr hal::byte address_byte = 0x00;

constexpr hal::byte cmd_send_kb_general_data = 0x02;
constexpr hal::byte cmd_send_kb_media_data = 0x03;
constexpr hal::byte cmd_send_ms_abs_data = 0x04;
constexpr hal::byte cmd_send_ms_rel_data = 0x05;

ch9329::ch9329(hal::serial& p_uart)
  : m_uart(&p_uart)
{
}

ch9329::mouse_absolute::mouse_absolute(std::uint16_t p_screen_width,
                                       std::uint16_t p_screen_height)
  : m_screen_width(p_screen_width)
  , m_screen_height(p_screen_height)

{
  m_data = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
}

ch9329::mouse_relative::mouse_relative()
{
  m_data = { 0x01, 0x00, 0x00, 0x00, 0x00 };
}

ch9329::keyboard_general::keyboard_general()
{
  m_data = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
}

hal::byte get_size_byte(hal::byte p_command)
{
  switch (p_command) {
    case cmd_send_kb_general_data:
      return 0x08;
    case cmd_send_kb_media_data:
      return 0x01;
    case cmd_send_ms_abs_data:
      return 0x07;
    case cmd_send_ms_rel_data:
      return 0x05;
  }
  return 0x00;
}

void send_start_bytes(serial& p_serial, hal::byte p_command)
{
  std::array<hal::byte, 5> start_bytes = {
    header_byte_1, header_byte_2, address_byte, p_command
  };
  start_bytes[4] = get_size_byte(p_command);
  hal::print(p_serial, start_bytes);
}

hal::byte calculate_sum(std::span<hal::byte> p_bytes, hal::byte p_command)
{
  std::uint8_t sum_byte = header_byte_1;
  sum_byte += header_byte_2;
  sum_byte += address_byte;
  sum_byte += p_command;
  sum_byte += get_size_byte(p_command);
  for (hal::byte byte : p_bytes) {
    sum_byte += byte;
  }
  return sum_byte;
}

void ch9329::send(ch9329::mouse_relative const& p_data)
{
  auto bytes = p_data.get_data();
  send_start_bytes(*m_uart, cmd_send_ms_rel_data);
  hal::print(*m_uart, bytes);
  auto sum_byte = calculate_sum(bytes, cmd_send_ms_rel_data);
  hal::print(*m_uart, std::to_array({ sum_byte }));
}

void ch9329::send(ch9329::mouse_absolute const& p_data)
{
  auto bytes = p_data.get_data();
  send_start_bytes(*m_uart, cmd_send_ms_abs_data);
  hal::print(*m_uart, bytes);
  auto sum_byte = calculate_sum(bytes, cmd_send_ms_abs_data);
  hal::print(*m_uart, std::to_array({ sum_byte }));
}

void ch9329::send(ch9329::keyboard_general const& p_data)
{
  auto bytes = p_data.get_data();
  send_start_bytes(*m_uart, cmd_send_kb_general_data);
  hal::print(*m_uart, bytes);
  auto sum_byte = calculate_sum(bytes, cmd_send_kb_general_data);
  hal::print(*m_uart, std::to_array({ sum_byte }));
}

// mouse absolute functions
ch9329::mouse_absolute& ch9329::mouse_absolute::position(std::uint16_t p_x,
                                                         std::uint16_t p_y)
{
  if (p_x >= m_screen_width) {
    p_x = m_screen_width - 1;
  }
  if (p_y >= m_screen_height) {
    p_y = m_screen_height - 1;
  }
  std::uint16_t x_value = ((p_x * 4096) / m_screen_width);
  hal::byte lower_bits = x_value & 0xFF;
  m_data[2] = lower_bits;
  hal::byte upper_bits = (x_value >> 8) & 0xFF;
  m_data[3] = upper_bits;

  std::uint16_t y_value = ((p_y * 4096) / m_screen_height);
  lower_bits = y_value & 0xFF;
  m_data[4] = lower_bits;
  upper_bits = (y_value >> 8) & 0xFF;
  m_data[5] = upper_bits;
  return *this;
}

ch9329::mouse_absolute& ch9329::mouse_absolute::scroll_position(
  std::int8_t p_scroll_offset)
{
  m_data[6] = p_scroll_offset;
  return *this;
}

ch9329::mouse_absolute& ch9329::mouse_absolute::left_button(bool p_pressed)
{
  constexpr auto left_button_mask = hal::bit_mask::from<0>();
  hal::bit_modify(m_data[1]).insert<left_button_mask>(p_pressed);
  return *this;
  return *this;
}

ch9329::mouse_absolute& ch9329::mouse_absolute::middle_button(bool p_pressed)
{
  constexpr auto middle_button_mask = hal::bit_mask::from<2>();
  hal::bit_modify(m_data[1]).insert<middle_button_mask>(p_pressed);
  return *this;
}

ch9329::mouse_absolute& ch9329::mouse_absolute::right_button(bool p_pressed)
{
  constexpr auto right_button_mask = hal::bit_mask::from<1>();
  hal::bit_modify(m_data[1]).insert<right_button_mask>(p_pressed);
  return *this;
}

// mouse relative functions
ch9329::mouse_relative& ch9329::mouse_relative::move(std::int8_t p_x_offset,
                                                     std::int8_t p_y_offset)
{
  m_data[2] = p_x_offset;
  m_data[3] = p_y_offset;
  return *this;
}

ch9329::mouse_relative& ch9329::mouse_relative::scroll_position(
  std::int8_t p_scroll_offset)
{
  m_data[4] = p_scroll_offset;
  return *this;
}

ch9329::mouse_relative& ch9329::mouse_relative::left_button(bool p_pressed)
{
  constexpr auto left_button_mask = hal::bit_mask::from<0>();
  hal::bit_modify(m_data[1]).insert<left_button_mask>(p_pressed);
  return *this;
}

ch9329::mouse_relative& ch9329::mouse_relative::middle_button(bool p_pressed)
{
  constexpr auto middle_button_mask = hal::bit_mask::from<2>();
  hal::bit_modify(m_data[1]).insert<middle_button_mask>(p_pressed);
  return *this;
}

ch9329::mouse_relative& ch9329::mouse_relative::right_button(bool p_pressed)
{
  constexpr auto right_button_mask = hal::bit_mask::from<1>();
  hal::bit_modify(m_data[1]).insert<right_button_mask>(p_pressed);
  return *this;
}

// keyboard general functions
ch9329::keyboard_general& ch9329::keyboard_general::press_control_key(
  control_key_bit p_key)
{
  hal::byte mask = 0x00;
  switch (p_key) {
    case control_key_bit::left_control:
      mask = 0b1;
      break;
    case control_key_bit::left_shift:
      mask = 0b10;
      break;
    case control_key_bit::left_alt:
      mask = 0b100;
      break;
    case control_key_bit::left_windows:
      mask = 0b1000;
      break;
    case control_key_bit::right_control:
      mask = 0b10000;
      break;
    case control_key_bit::right_shift:
      mask = 0b100000;
      break;
    case control_key_bit::right_alt:
      mask = 0b1000000;
      break;
    case control_key_bit::right_windows:
      mask = 0b10000000;
      break;
  }
  m_data[0] = m_data[0] | mask;
  return *this;
}

ch9329::keyboard_general& ch9329::keyboard_general::release_control_key(
  control_key_bit p_key)
{
  hal::byte mask = 0xFF;
  switch (p_key) {
    case control_key_bit::left_control:
      mask = 0b11111110;
      break;
    case control_key_bit::left_shift:
      mask = 0b11111101;
      break;
    case control_key_bit::left_alt:
      mask = 0b11111011;
      break;
    case control_key_bit::left_windows:
      mask = 0b11110111;
      break;
    case control_key_bit::right_control:
      mask = 0b11101111;
      break;
    case control_key_bit::right_shift:
      mask = 0b11011111;
      break;
    case control_key_bit::right_alt:
      mask = 0b10111111;
      break;
    case control_key_bit::right_windows:
      mask = 0b01111111;
      break;
  }
  m_data[0] = m_data[0] & mask;
  return *this;
}

ch9329::keyboard_general& ch9329::keyboard_general::press_normal_key(
  normal_key p_key,
  uint8_t p_slot)
{
  if (p_slot < 1) {
    p_slot = 1;
  } else if (p_slot > 6) {
    p_slot = 6;
  }
  m_data[p_slot + 1] = static_cast<hal::byte>(p_key);
  return *this;
}

ch9329::keyboard_general& ch9329::keyboard_general::release_normal_key(
  normal_key p_key)
{
  for (int i = 2; i < 8; i++) {
    if (m_data[i] == static_cast<hal::byte>(p_key)) {
      m_data[i] = 0x00;
    }
  }
  return *this;
}

ch9329::keyboard_general& ch9329::keyboard_general::release_all_keys()
{
  m_data = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  return *this;
}
}  // namespace hal::input
