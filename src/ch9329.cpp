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

#include <cstdint>

#include <algorithm>
#include <array>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/steady_clock.hpp>
#include <span>

#include <libhal-input/ch9329.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/serial.hpp>
#include <libhal/serial.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>
#include <string_view>

namespace hal::input {

constexpr hal::byte header_byte_1 = 0x57;
constexpr hal::byte header_byte_2 = 0xAB;
constexpr hal::byte address_byte = 0x00;

constexpr hal::byte cmd_get_info = 0x01;
constexpr hal::byte cmd_get_usb_string = 0x0A;
constexpr hal::byte cmd_set_usb_string = 0x0B;
constexpr hal::byte cmd_restore_factory_default_settings = 0x0C;
constexpr hal::byte cmd_reset = 0x0F;
constexpr hal::byte cmd_send_kb_general_data = 0x02;
constexpr hal::byte cmd_send_kb_media_data = 0x03;
constexpr hal::byte cmd_send_ms_abs_data = 0x04;
constexpr hal::byte cmd_send_ms_rel_data = 0x05;
constexpr hal::byte cmd_get_para_cfg = 0x08;
constexpr hal::byte cmd_set_para_cfg = 0x09;

constexpr auto header_frame_size = 5;
constexpr auto info_response_length = 14;
constexpr auto response_length = 7;
constexpr auto response_byte = 5;
constexpr auto response_header_length = 5;
constexpr auto check_sum_read_length = 1;
constexpr auto string_header_length = 2;
constexpr auto parameters_length = 50;

constexpr std::array<hertz, 5> standard_baud_rates = { 9600,
                                                       19200,
                                                       38400,
                                                       57600,
                                                       115200 };

ch9329::ch9329(hal::serial& p_uart, hal::steady_clock& p_clock)
  : m_uart(&p_uart)
{
  using namespace std::chrono_literals;
  auto wait_time = hal::create_timeout(p_clock, 10ms);
  for (auto baud : standard_baud_rates) {
    try {
      m_uart->configure({ .baud_rate = baud });
      auto response = get_info(wait_time);
      if (response.version < 0x30) {
        break;
      }
    } catch (timed_out) {
      // what do here?
      throw;
    }
  }
}

ch9329::ch9329_parameters::ch9329_parameters(
  std::array<hal::byte, parameters_length> p_config_bytes)
  : m_config_bytes(p_config_bytes){};

ch9329::ch9329_parameters::ch9329_parameters()
{
  // these are the default parameters
  m_config_bytes = {
    0x80, 0x80, 0x00, 0x00, 0x00, 0x25, 0x80, 0x08, 0x00, 0x00,
    0x03, 0x86, 0x1A, 0x29, 0xE1, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
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

ch9329::keyboard_media::keyboard_media()
{
  m_data = { 0x02, 0x00, 0x00, 0x00 };
}

ch9329::keyboard_acpi::keyboard_acpi()
{
  m_data = { 0x01, 0x00 };
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
      return 0x04;
    case cmd_send_ms_abs_data:
      return 0x07;
    case cmd_send_ms_rel_data:
      return 0x05;
    case cmd_get_usb_string:
      return 0x01;
    case cmd_set_para_cfg:
      return 0x32;
  }
  return 0x00;
}

void send_start_bytes(serial& p_serial,
                      hal::byte p_command,
                      std::uint8_t p_size = 0)
{
  std::array<hal::byte, header_frame_size> start_bytes = {
    header_byte_1, header_byte_2, address_byte, p_command
  };
  if (p_size == 0) {
    start_bytes[4] = get_size_byte(p_command);
  } else {
    start_bytes[4] = p_size;
  }
  hal::print(p_serial, start_bytes);
}

hal::byte calculate_sum(std::span<hal::byte> p_bytes,
                        hal::byte p_command,
                        std::uint8_t p_custom_length = 0)
{
  std::uint8_t sum_byte = header_byte_1;
  sum_byte += header_byte_2;
  sum_byte += address_byte;
  sum_byte += p_command;
  sum_byte += get_size_byte(p_command);
  for (hal::byte byte : p_bytes) {
    sum_byte += byte;
  }
  sum_byte += p_custom_length;
  return sum_byte;
}

void send_command_with_bytes(std::span<hal::byte> p_bytes,
                             hal::byte p_command,
                             serial& p_serial,
                             std::uint8_t p_size = 0)
{
  send_start_bytes(p_serial, p_command, p_size);
  hal::print(p_serial, p_bytes);
  auto sum_byte = calculate_sum(p_bytes, p_command, p_size);
  hal::print(p_serial, std::to_array({ sum_byte }));
}

void ch9329::send(ch9329::mouse_relative const& p_data)
{
  auto bytes = p_data.get_data();
  send_command_with_bytes(bytes, cmd_send_ms_rel_data, *m_uart);
}

void ch9329::send(ch9329::mouse_absolute const& p_data)
{
  auto bytes = p_data.get_data();
  send_command_with_bytes(bytes, cmd_send_ms_abs_data, *m_uart);
}

void ch9329::send(ch9329::keyboard_general const& p_data)
{
  auto bytes = p_data.get_data();
  send_command_with_bytes(bytes, cmd_send_kb_general_data, *m_uart);
}

void ch9329::send(keyboard_media const& p_data)
{
  auto bytes = p_data.get_data();
  send_command_with_bytes(bytes, cmd_send_kb_media_data, *m_uart);
}

void ch9329::send(keyboard_acpi const& p_data)
{
  auto bytes = p_data.get_data();
  send_command_with_bytes(bytes, cmd_send_kb_media_data, *m_uart, 2);
}

ch9329::chip_info ch9329::get_info(
  hal::function_ref<hal::timeout_function> p_timeout = hal::never_timeout())
{
  std::array<hal::byte, info_response_length> response{};
  send_start_bytes(*m_uart, cmd_get_info);
  hal::print(*m_uart, std::to_array({ (hal::byte)0x03 }));  // sum byte
  hal::read(*m_uart, response, p_timeout);
  chip_info info = {};
  info.version = response[5];
  info.enumeration_status = response[6];
  info.num_lock = response[7] & (1 << hal::value(lock_status::num_lock));
  info.caps_lock = response[7] & (1 << hal::value(lock_status::caps_lock));
  info.scroll_lock = response[7] & (1 << hal::value(lock_status::scroll_lock));
  return info;
}

ch9329::usb_string_descriptor get_usb_string_descriptor(
  serial& p_serial,
  hal::byte p_string_type,
  hal::function_ref<hal::timeout_function> p_timeout)
{
  ch9329::usb_string_descriptor str;
  auto bytes = std::to_array({ p_string_type });
  send_command_with_bytes(bytes, cmd_get_usb_string, p_serial);
  std::array<hal::byte, response_length> header_bytes;
  hal::read(p_serial, header_bytes, p_timeout);
  str.length = header_bytes.back();
  hal::read(p_serial, str.received_data(), p_timeout);
  std::array<hal::byte, check_sum_read_length> check_sum;
  hal::read(p_serial, check_sum, p_timeout);
  return str;
}

hal::byte set_usb_string_descriptor(serial& p_serial,
                                    hal::byte p_string_type,
                                    std::string_view p_string)
{
  uint8_t string_length = p_string.length();
  auto string_header = std::to_array({ p_string_type, string_length });
  send_start_bytes(
    p_serial, cmd_set_usb_string, string_length + string_header_length);
  hal::print(p_serial, string_header);
  hal::print(p_serial, p_string);
  // get the sum up to the end of the string header
  auto sum_byte = calculate_sum(string_header, cmd_set_usb_string);
  // add string header bytes to sum
  sum_byte += (string_length + string_header_length);
  // add chars in string to sum
  for (hal::byte byte : p_string) {
    sum_byte += byte;
  }
  // send sum byte
  hal::print(p_serial, std::to_array({ sum_byte }));
  std::array<hal::byte, response_length> response;
  hal::read(p_serial, response, hal::never_timeout());
  return response[response_byte];
}

ch9329::usb_string_descriptor ch9329::get_manufacturer_descriptor(
  hal::function_ref<hal::timeout_function> p_timeout = hal::never_timeout())
{
  return get_usb_string_descriptor(*m_uart, 0x00, p_timeout);
}

ch9329::usb_string_descriptor ch9329::get_product_descriptor(
  hal::function_ref<hal::timeout_function> p_timeout = hal::never_timeout())
{
  return get_usb_string_descriptor(*m_uart, 0x01, p_timeout);
}

ch9329::usb_string_descriptor ch9329::get_serial_number_descriptor(
  hal::function_ref<hal::timeout_function> p_timeout = hal::never_timeout())
{
  return get_usb_string_descriptor(*m_uart, 0x02, p_timeout);
}

hal::byte ch9329::set_manufacturer_descriptor(std::string_view p_string)
{
  return set_usb_string_descriptor(*m_uart, 0x00, p_string);
}

hal::byte ch9329::set_product_descriptor(std::string_view p_string)
{
  return set_usb_string_descriptor(*m_uart, 0x01, p_string);
}

hal::byte ch9329::set_serial_number_descriptor(std::string_view p_string)
{
  return set_usb_string_descriptor(*m_uart, 0x02, p_string);
}

hal::byte ch9329::restore_factory_default_settings()
{
  send_start_bytes(*m_uart, cmd_restore_factory_default_settings);
  auto sum_byte = calculate_sum({}, cmd_restore_factory_default_settings);
  hal::print(*m_uart, std::to_array({ sum_byte }));
  std::array<hal::byte, response_length> response;
  hal::read(*m_uart, response, hal::never_timeout());
  return response[response_byte];
}

hal::byte ch9329::reset()
{
  send_start_bytes(*m_uart, cmd_reset);
  auto sum_byte = calculate_sum({}, cmd_reset);
  hal::print(*m_uart, std::to_array({ sum_byte }));
  std::array<hal::byte, response_length> response;
  hal::read(*m_uart, response, hal::never_timeout());
  return response[response_byte];
}

ch9329::ch9329_parameters ch9329::get_parameters(
  hal::function_ref<hal::timeout_function> p_timeout = hal::never_timeout())
{
  send_start_bytes(*m_uart, cmd_get_para_cfg);
  auto sum_byte = calculate_sum({}, cmd_get_para_cfg);
  hal::print(*m_uart, std::to_array({ sum_byte }));
  std::array<hal::byte, response_header_length> response_header;
  std::array<hal::byte, parameters_length> response;
  std::array<hal::byte, check_sum_read_length> response_sum;

  hal::read(*m_uart, response_header, p_timeout);
  hal::read(*m_uart, response, p_timeout);
  hal::read(*m_uart, response_sum, p_timeout);

  ch9329_parameters params(response);

  return params;
}

hal::byte ch9329::set_parameters(
  ch9329_parameters const& p_parameters,
  hal::function_ref<hal::timeout_function> p_timeout = hal::never_timeout())
{
  auto bytes = p_parameters.get_config_bytes();
  send_command_with_bytes(bytes, cmd_set_para_cfg, *m_uart);
  std::array<hal::byte, response_length> response;
  hal::read(*m_uart, response, p_timeout);
  return response[response_byte];
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

// keyboard media functions
ch9329::keyboard_media& ch9329::keyboard_media::press_media_key(media_key p_key)
{
  // calculate which byte the bit to change is in
  auto byte_num = (static_cast<uint8_t>(p_key) >> 3) + 1;
  // get which bit to change by reading last 3 bits
  auto bit_num = static_cast<uint8_t>(p_key) & 0b111;
  // change the byte/bit combo to 1
  m_data[byte_num] |= 1 << bit_num;
  return *this;
}

ch9329::keyboard_media& ch9329::keyboard_media::release_media_key(
  media_key p_key)
{
  // calculate which byte the bit to change is in
  auto byte_num = (static_cast<uint8_t>(p_key) >> 3) + 1;
  // get which bit to change by reading last 3 bits
  auto bit_num = static_cast<uint8_t>(p_key) & 0b111;
  // change the byte/bit combo to 0
  m_data[byte_num] &= ~(1 << bit_num);
  return *this;
}

ch9329::keyboard_media& ch9329::keyboard_media::release_all_keys()
{
  m_data = { 0x01, 0x00, 0x00, 0x00 };
  return *this;
}

// keyboard acpi functions
ch9329::keyboard_acpi& ch9329::keyboard_acpi::press_acpi_key(acpi_key p_key)
{
  auto bit_num = static_cast<uint8_t>(p_key);
  m_data[1] |= 1 << bit_num;
  return *this;
}

ch9329::keyboard_acpi& ch9329::keyboard_acpi::release_acpi_key(acpi_key p_key)
{
  auto bit_num = static_cast<uint8_t>(p_key);
  m_data[1] &= ~(1 << bit_num);
  return *this;
}

ch9329::keyboard_acpi& ch9329::keyboard_acpi::release_all_keys()
{
  m_data = { 0x02, 0x00 };
  return *this;
}

// keyboard general functions
ch9329::keyboard_general& ch9329::keyboard_general::press_control_key(
  control_key p_key)
{

  hal::byte mask = (1 << hal::value(p_key));
  m_data[0] = m_data[0] | mask;
  return *this;
}

ch9329::keyboard_general& ch9329::keyboard_general::release_control_key(
  control_key p_key)
{
  hal::byte mask = ~(1 << hal::value(p_key));
  m_data[0] = m_data[0] & mask;
  return *this;
}

ch9329::keyboard_general& ch9329::keyboard_general::press_normal_key(
  normal_key p_key,
  uint8_t p_slot)
{
  constexpr std::uint8_t slot_min = 1;
  constexpr std::uint8_t slot_max = 6;
  p_slot = std::clamp(p_slot, slot_min, slot_max);
  // first byte of m_data is reserved, first slot starts at position 2
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

ch9329::ch9329_parameters& ch9329::ch9329_parameters::set_config_bytes(
  std::array<hal::byte, parameters_length> p_config_bytes)
{
  m_config_bytes = p_config_bytes;
  return *this;
};

ch9329::ch9329_parameters& ch9329::ch9329_parameters::set_chip_working_mode(
  working_mode p_working_mode)
{
  m_config_bytes[0] = static_cast<hal::byte>(p_working_mode);
  return *this;
};
ch9329::ch9329_parameters&
ch9329::ch9329_parameters::set_serial_communication_mode(
  communication_mode p_communication_mode)
{
  m_config_bytes[1] = static_cast<hal::byte>(p_communication_mode);
  return *this;
};
ch9329::ch9329_parameters& ch9329::ch9329_parameters::set_serial_address(
  hal::byte p_serial)
{
  m_config_bytes[2] = p_serial;
  return *this;
};
ch9329::ch9329_parameters& ch9329::ch9329_parameters::set_serial_mode_baud_rate(
  std::uint32_t p_baud)
{
  m_config_bytes[3] = static_cast<hal::byte>(p_baud >> 24);
  m_config_bytes[4] = static_cast<hal::byte>(p_baud >> 16);
  m_config_bytes[5] = static_cast<hal::byte>(p_baud >> 8);
  m_config_bytes[6] = static_cast<hal::byte>(p_baud);
  return *this;
};
ch9329::ch9329_parameters&
ch9329::ch9329_parameters::set_serial_mode_packet_interval(
  std::uint16_t p_packet_interval)
{
  m_config_bytes[9] = static_cast<hal::byte>(p_packet_interval >> 8);
  m_config_bytes[10] = static_cast<hal::byte>(p_packet_interval);
  return *this;
};
ch9329::ch9329_parameters& ch9329::ch9329_parameters::set_vendor_id(
  std::uint16_t p_vid)
{
  m_config_bytes[11] = static_cast<hal::byte>(p_vid >> 8);
  m_config_bytes[12] = static_cast<hal::byte>(p_vid);
  return *this;
};
ch9329::ch9329_parameters& ch9329::ch9329_parameters::set_product_id(
  std::uint16_t p_id)
{
  m_config_bytes[13] = static_cast<hal::byte>(p_id >> 8);
  m_config_bytes[14] = static_cast<hal::byte>(p_id);
  return *this;
};
ch9329::ch9329_parameters&
ch9329::ch9329_parameters::set_ascii_mode_kb_upload_interval(
  std::uint16_t p_upload_interval)
{
  m_config_bytes[15] = static_cast<hal::byte>(p_upload_interval >> 8);
  m_config_bytes[16] = static_cast<hal::byte>(p_upload_interval);
  return *this;
};
ch9329::ch9329_parameters&
ch9329::ch9329_parameters::set_ascii_mode_kb_release_delay(
  std::uint16_t p_release_dealy)
{
  m_config_bytes[17] = static_cast<hal::byte>(p_release_dealy >> 8);
  m_config_bytes[18] = static_cast<hal::byte>(p_release_dealy);
  return *this;
};
ch9329::ch9329_parameters&
ch9329::ch9329_parameters::set_ascii_mode_kb_auto_enter(bool p_auto_enter)
{
  m_config_bytes[19] =
    static_cast<hal::byte>(p_auto_enter);  // TODO: does this work?
  return *this;
};
ch9329::ch9329_parameters&
ch9329::ch9329_parameters::set_ascii_mode_kb_carriage_return_1(
  std::uint32_t p_carriage_return)
{
  m_config_bytes[20] = static_cast<hal::byte>(p_carriage_return >> 24);
  m_config_bytes[21] = static_cast<hal::byte>(p_carriage_return >> 16);
  m_config_bytes[22] = static_cast<hal::byte>(p_carriage_return >> 8);
  m_config_bytes[23] = static_cast<hal::byte>(p_carriage_return);
  return *this;
};
ch9329::ch9329_parameters&
ch9329::ch9329_parameters::set_ascii_mode_kb_carriage_return_2(
  std::uint32_t p_carriage_return)
{
  m_config_bytes[24] = static_cast<hal::byte>(p_carriage_return >> 24);
  m_config_bytes[25] = static_cast<hal::byte>(p_carriage_return >> 16);
  m_config_bytes[26] = static_cast<hal::byte>(p_carriage_return >> 8);
  m_config_bytes[27] = static_cast<hal::byte>(p_carriage_return);
  return *this;
};
ch9329::ch9329_parameters& ch9329::ch9329_parameters::set_kb_start_filter_chars(
  std::array<hal::byte, 4> p_start_filters)
{
  m_config_bytes[28] = p_start_filters[0];
  m_config_bytes[29] = p_start_filters[1];
  m_config_bytes[30] = p_start_filters[2];
  m_config_bytes[31] = p_start_filters[3];
  return *this;
};
ch9329::ch9329_parameters& ch9329::ch9329_parameters::set_kb_end_filter_chars(
  std::array<hal::byte, 4> p_end_filters)
{
  m_config_bytes[32] = p_end_filters[0];
  m_config_bytes[33] = p_end_filters[1];
  m_config_bytes[34] = p_end_filters[2];
  m_config_bytes[35] = p_end_filters[3];
  return *this;
};
ch9329::ch9329_parameters& ch9329::ch9329_parameters::set_usb_string_enable(
  custom_descriptor p_descriptor,
  bool p_enable)
{
  if (p_enable) {
    hal::byte mask = 1 << hal::value(p_descriptor);
    m_config_bytes[36] = m_config_bytes[36] | mask;
  } else {
    hal::byte mask = ~(1 << hal::value(p_descriptor));
    m_config_bytes[36] = m_config_bytes[36] & mask;
  }
  return *this;
};
ch9329::ch9329_parameters&
ch9329::ch9329_parameters::set_ascii_mode_kb_fast_upload_mode(
  bool p_fast_upload)
{
  m_config_bytes[37] = static_cast<hal::byte>(p_fast_upload);
  return *this;
};
working_mode ch9329::ch9329_parameters::get_chip_working_mode()
{
  return static_cast<working_mode>(m_config_bytes[0]);
};
communication_mode ch9329::ch9329_parameters::get_serial_communication_mode()
{
  return static_cast<communication_mode>(m_config_bytes[1]);
};
hal::byte ch9329::ch9329_parameters::get_serial_address()
{
  return m_config_bytes[2];
};
std::uint32_t ch9329::ch9329_parameters::get_serial_mode_baud_rate()
{
  std::uint32_t baud = m_config_bytes[3];
  baud = (baud << 8) | m_config_bytes[4];
  baud = (baud << 8) | m_config_bytes[5];
  baud = (baud << 8) | m_config_bytes[6];
  return baud;
};
std::uint16_t ch9329::ch9329_parameters::get_serial_mode_packet_interval()
{
  return static_cast<uint16_t>((m_config_bytes[9] << 8) | m_config_bytes[10]);
};
std::uint16_t ch9329::ch9329_parameters::get_vendor_id()
{
  return static_cast<uint16_t>((m_config_bytes[11] << 8) | m_config_bytes[12]);
};
std::uint16_t ch9329::ch9329_parameters::get_product_id()
{
  return static_cast<uint16_t>((m_config_bytes[13] << 8) | m_config_bytes[14]);
};
std::uint16_t ch9329::ch9329_parameters::get_ascii_mode_kb_upload_interval()
{
  return static_cast<uint16_t>((m_config_bytes[15] << 8) | m_config_bytes[16]);
};
std::uint16_t ch9329::ch9329_parameters::get_ascii_mode_kb_release_delay()
{
  return static_cast<uint16_t>((m_config_bytes[17] << 8) | m_config_bytes[18]);
};
bool ch9329::ch9329_parameters::get_ascii_mode_kb_auto_enter()
{
  return static_cast<bool>(m_config_bytes[19]);
};
std::uint32_t ch9329::ch9329_parameters::get_ascii_mode_kb_carriage_return_1()
{
  std::uint32_t carriage_ret_1 = m_config_bytes[20];
  carriage_ret_1 = (carriage_ret_1 << 8) | m_config_bytes[21];
  carriage_ret_1 = (carriage_ret_1 << 8) | m_config_bytes[22];
  carriage_ret_1 = (carriage_ret_1 << 8) | m_config_bytes[23];
  return carriage_ret_1;
};
std::uint32_t ch9329::ch9329_parameters::get_ascii_mode_kb_carriage_return_2()
{
  std::uint32_t carriage_ret_2 = m_config_bytes[24];
  carriage_ret_2 = (carriage_ret_2 << 8) | m_config_bytes[25];
  carriage_ret_2 = (carriage_ret_2 << 8) | m_config_bytes[26];
  carriage_ret_2 = (carriage_ret_2 << 8) | m_config_bytes[27];
  return carriage_ret_2;
};
std::array<hal::byte, 4> ch9329::ch9329_parameters::get_kb_start_filter_chars()
{
  std::array<hal::byte, 4> start_filters = { m_config_bytes[28],
                                             m_config_bytes[29],
                                             m_config_bytes[30],
                                             m_config_bytes[31] };
  return start_filters;
};
std::array<hal::byte, 4> ch9329::ch9329_parameters::get_kb_end_filter_chars()
{
  std::array<hal::byte, 4> end_filters = { m_config_bytes[32],
                                           m_config_bytes[33],
                                           m_config_bytes[34],
                                           m_config_bytes[35] };
  return end_filters;
};
bool ch9329::ch9329_parameters::get_usb_string_enable(
  custom_descriptor p_descriptor)
{
  hal::byte mask = 1 << hal::value(p_descriptor);
  hal::byte status = (m_config_bytes[36] | mask) >> hal::value(p_descriptor);
  return static_cast<bool>(status);
};
bool ch9329::ch9329_parameters::get_ascii_mode_kb_fast_upload_mode()
{
  return static_cast<bool>(m_config_bytes[37]);
};
}  // namespace hal::input
