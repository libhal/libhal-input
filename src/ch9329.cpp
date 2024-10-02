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

#include <array>
#include <cstdint>
#include <libhal-input/ch9329.hpp>
#include <libhal-util/bit.hpp>
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

ch9329::ch9329(hal::serial& p_uart)
  : m_uart(&p_uart)
{
}

ch9329::ch9329_parameters::ch9329_parameters(
  std::array<hal::byte, 50> p_config_bytes)
  : m_config_bytes(p_config_bytes)
{
  std::uint32_t baud = m_config_bytes[3];
  baud = (baud << 8) | m_config_bytes[4];
  baud = (baud << 8) | m_config_bytes[5];
  baud = (baud << 8) | m_config_bytes[6];

  std::uint32_t carriage_ret_1 = m_config_bytes[20];
  carriage_ret_1 = (carriage_ret_1 << 8) | m_config_bytes[21];
  carriage_ret_1 = (carriage_ret_1 << 8) | m_config_bytes[22];
  carriage_ret_1 = (carriage_ret_1 << 8) | m_config_bytes[23];

  std::uint32_t carriage_ret_2 = m_config_bytes[24];
  carriage_ret_2 = (carriage_ret_2 << 8) | m_config_bytes[25];
  carriage_ret_2 = (carriage_ret_2 << 8) | m_config_bytes[26];
  carriage_ret_2 = (carriage_ret_2 << 8) | m_config_bytes[27];

  std::uint32_t start_filters = m_config_bytes[28];
  start_filters = (start_filters << 8) | m_config_bytes[29];
  start_filters = (start_filters << 8) | m_config_bytes[30];
  start_filters = (start_filters << 8) | m_config_bytes[31];

  std::uint32_t end_filters = m_config_bytes[32];
  end_filters = (end_filters << 8) | m_config_bytes[33];
  end_filters = (end_filters << 8) | m_config_bytes[34];
  end_filters = (end_filters << 8) | m_config_bytes[35];

  ch9329_parameters params(m_config_bytes);
  params.chip_working_mode = m_config_bytes[0];
  params.serial_communication_mode = m_config_bytes[1];
  params.serial_address = m_config_bytes[2];
  params.serial_mode_baud_rate = baud;
  params.serial_mode_packet_interval =
    static_cast<uint16_t>((m_config_bytes[9] << 8) | m_config_bytes[10]);
  params.vendor_id =
    static_cast<uint16_t>((m_config_bytes[11] << 8) | m_config_bytes[12]);
  params.p_id =
    static_cast<uint16_t>((m_config_bytes[13] << 8) | m_config_bytes[14]);
  params.ascii_mode_kb_upload_interval =
    static_cast<uint16_t>((m_config_bytes[15] << 8) | m_config_bytes[16]);
  params.ascii_mode_kb_release_delay =
    static_cast<uint16_t>((m_config_bytes[17] << 8) | m_config_bytes[18]);
  params.ascii_mode_kb_auto_enter = m_config_bytes[19];
  params.ascii_mode_kb_carriage_return_1 = carriage_ret_1;
  params.ascii_mode_kb_carriage_return_2 = carriage_ret_2;
  params.kb_start_filter_chars = start_filters;
  params.kb_end_filter_chars = end_filters;
  params.usb_string_enable = m_config_bytes[36];
  params.ascii_mode_kb_fast_upload_mode = m_config_bytes[37];
};

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
  }
  return 0x00;
}

void send_start_bytes(serial& p_serial, hal::byte p_command, uint8_t p_size = 0)
{
  std::array<hal::byte, 5> start_bytes = {
    header_byte_1, header_byte_2, address_byte, p_command
  };
  if (p_size == 0) {
    start_bytes[4] = get_size_byte(p_command);
  } else {
    start_bytes[4] = p_size;
  }
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

void ch9329::send(keyboard_media const& p_data)
{
  auto bytes = p_data.get_data();
  send_start_bytes(*m_uart, cmd_send_kb_media_data);
  hal::print(*m_uart, bytes);
  auto sum_byte = calculate_sum(bytes, cmd_send_kb_media_data);
  hal::print(*m_uart, std::to_array({ sum_byte }));
}

void ch9329::send(keyboard_acpi const& p_data)
{
  auto bytes = p_data.get_data();
  send_start_bytes(*m_uart, cmd_send_kb_media_data, 2);
  hal::print(*m_uart, bytes);
  auto sum_byte = calculate_sum(bytes, cmd_send_kb_media_data);
  hal::print(*m_uart, std::to_array({ sum_byte }));
}

ch9329::chip_info ch9329::get_info()
{
  std::array<hal::byte, 14> response{};
  send_start_bytes(*m_uart, cmd_get_info);
  hal::print(*m_uart, std::to_array({ (hal::byte)0x03 }));  // sum byte
  hal::read(*m_uart, response, hal::never_timeout());
  chip_info info = {};
  info.version = response[5];
  info.enumeration_status = response[6];
  info.num_lock = response[7] & 0b1;
  info.caps_lock = response[7] & 0b10;
  info.scroll_lock = response[7] & 0b100;
  return info;
}

ch9329::usb_string_descriptor get_usb_string_descriptor(serial& p_serial,
                                                        hal::byte p_string_type)
{
  ch9329::usb_string_descriptor str;
  auto bytes = std::to_array({ p_string_type });
  send_start_bytes(p_serial, cmd_get_usb_string);
  hal::print(p_serial, bytes);
  auto sum_byte = calculate_sum(bytes, cmd_get_usb_string);
  hal::print(p_serial, std::to_array({ sum_byte }));
  std::array<hal::byte, 7> header_bytes;
  hal::read(p_serial, header_bytes, hal::never_timeout());
  str.length = header_bytes[6];
  hal::read(p_serial, str.received_data(), hal::never_timeout());
  std::array<hal::byte, 1> check_sum;
  hal::read(p_serial, check_sum, hal::never_timeout());
  return str;
}

hal::byte set_usb_string_descriptor(serial& p_serial,
                                    hal::byte p_string_type,
                                    std::string_view p_string)
{
  uint8_t string_length = p_string.length();
  auto bytes = std::to_array({ p_string_type, string_length });
  send_start_bytes(p_serial, cmd_set_usb_string, string_length + 2);
  hal::print(p_serial, bytes);
  hal::print(p_serial, p_string);
  auto sum_byte = calculate_sum(bytes, cmd_set_usb_string);
  sum_byte += (string_length + 2);
  for (hal::byte byte : p_string) {
    sum_byte += byte;
  }
  hal::print(p_serial, std::to_array({ sum_byte }));
  std::array<hal::byte, 7> response;
  hal::read(p_serial, response, hal::never_timeout());
  return response[5];
}

ch9329::usb_string_descriptor ch9329::get_manufacturer_descriptor()
{
  return get_usb_string_descriptor(*m_uart, 0x00);
}

ch9329::usb_string_descriptor ch9329::get_product_descriptor()
{
  return get_usb_string_descriptor(*m_uart, 0x01);
}

ch9329::usb_string_descriptor ch9329::get_serial_number_descriptor()
{
  return get_usb_string_descriptor(*m_uart, 0x02);
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
  std::array<hal::byte, 7> response;
  hal::read(*m_uart, response, hal::never_timeout());
  return response[5];
}

hal::byte ch9329::reset()
{
  send_start_bytes(*m_uart, cmd_reset);
  auto sum_byte = calculate_sum({}, cmd_reset);
  hal::print(*m_uart, std::to_array({ sum_byte }));
  std::array<hal::byte, 7> response;
  hal::read(*m_uart, response, hal::never_timeout());
  return response[5];
}

ch9329::ch9329_parameters ch9329::get_parameters()
{
  send_start_bytes(*m_uart, cmd_get_para_cfg);
  auto sum_byte = calculate_sum({}, cmd_get_para_cfg);
  hal::print(*m_uart, std::to_array({ sum_byte }));
  std::array<hal::byte, 5> response_header;
  std::array<hal::byte, 50> response;
  std::array<hal::byte, 1> response_sum;

  hal::read(*m_uart, response_header, hal::never_timeout());
  hal::read(*m_uart, response, hal::never_timeout());
  hal::read(*m_uart, response_sum, hal::never_timeout());

  ch9329_parameters params(response);

  return params;
}

hal::byte ch9329::set_parameters(ch9329_parameters const& p_parameters)
{
  std::array<hal::byte, 50> bytes = {
    p_parameters.chip_working_mode,
    p_parameters.serial_communication_mode,
    p_parameters.serial_address,
    static_cast<hal::byte>(p_parameters.serial_mode_baud_rate >> 24),
    static_cast<hal::byte>(p_parameters.serial_mode_baud_rate >> 16),
    static_cast<hal::byte>(p_parameters.serial_mode_baud_rate >> 8),
    static_cast<hal::byte>(p_parameters.serial_mode_baud_rate),
    0x00,
    0x00,
    static_cast<hal::byte>(p_parameters.serial_mode_packet_interval >> 8),
    static_cast<hal::byte>(p_parameters.serial_mode_packet_interval),
    static_cast<hal::byte>(p_parameters.vendor_id >> 8),
    static_cast<hal::byte>(p_parameters.vendor_id),
    static_cast<hal::byte>(p_parameters.p_id >> 8),
    static_cast<hal::byte>(p_parameters.p_id),
    static_cast<hal::byte>(p_parameters.ascii_mode_kb_upload_interval >> 8),
    static_cast<hal::byte>(p_parameters.ascii_mode_kb_upload_interval),
    static_cast<hal::byte>(p_parameters.ascii_mode_kb_release_delay >> 8),
    static_cast<hal::byte>(p_parameters.ascii_mode_kb_release_delay),
    p_parameters.ascii_mode_kb_auto_enter,
    static_cast<hal::byte>(p_parameters.ascii_mode_kb_carriage_return_1 >> 24),
    static_cast<hal::byte>(p_parameters.ascii_mode_kb_carriage_return_1 >> 16),
    static_cast<hal::byte>(p_parameters.ascii_mode_kb_carriage_return_1 >> 8),
    static_cast<hal::byte>(p_parameters.ascii_mode_kb_carriage_return_1),
    static_cast<hal::byte>(p_parameters.ascii_mode_kb_carriage_return_2 >> 24),
    static_cast<hal::byte>(p_parameters.ascii_mode_kb_carriage_return_2 >> 16),
    static_cast<hal::byte>(p_parameters.ascii_mode_kb_carriage_return_2 >> 8),
    static_cast<hal::byte>(p_parameters.ascii_mode_kb_carriage_return_2),
    static_cast<hal::byte>(p_parameters.kb_start_filter_chars >> 24),
    static_cast<hal::byte>(p_parameters.kb_start_filter_chars >> 16),
    static_cast<hal::byte>(p_parameters.kb_start_filter_chars >> 8),
    static_cast<hal::byte>(p_parameters.kb_start_filter_chars),
    static_cast<hal::byte>(p_parameters.kb_end_filter_chars >> 24),
    static_cast<hal::byte>(p_parameters.kb_end_filter_chars >> 16),
    static_cast<hal::byte>(p_parameters.kb_end_filter_chars >> 8),
    static_cast<hal::byte>(p_parameters.kb_end_filter_chars),
    p_parameters.usb_string_enable,
    p_parameters.ascii_mode_kb_fast_upload_mode,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
  };
  // auto bytes = p_parameters.get_config_bytes();
  send_start_bytes(*m_uart, cmd_set_para_cfg, 50);
  hal::print(*m_uart, bytes);
  auto sum_byte = calculate_sum(bytes, cmd_set_para_cfg);
  hal::print(*m_uart, std::to_array({ sum_byte }));
  std::array<hal::byte, 7> response;
  hal::read(*m_uart, response, hal::never_timeout());
  return response[5];
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

// keyboard media functions
ch9329::keyboard_media& ch9329::keyboard_media::press_media_key(media_key p_key)
{
  auto byte_num = (static_cast<uint8_t>(p_key) >> 3) + 1;
  auto bit_num = static_cast<uint8_t>(p_key) & 0b111;
  m_data[byte_num] |= 1 << bit_num;
  return *this;
}

ch9329::keyboard_media& ch9329::keyboard_media::release_media_key(
  media_key p_key)
{
  auto byte_num = (static_cast<uint8_t>(p_key) >> 3) + 1;
  auto bit_num = static_cast<uint8_t>(p_key) & 0b111;
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
