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

#pragma once

#include <array>
#include <cstdint>
#include <libhal-input/ch9329_kb_bytes.hpp>
#include <libhal/serial.hpp>
#include <libhal/units.hpp>
#include <span>

namespace hal::input {
/**
 * @brief Driver for CH9329 UART to USB
 *
 */
class ch9329
{

public:
  struct chip_info
  {
    hal::byte version;
    bool enumeration_status;
    bool num_lock;
    bool caps_lock;
    bool scroll_lock;
  };

  struct usb_string_descriptor
  {
    auto full_span()
    {
      return std::span(buffer);
    }
    auto received_data()
    {
      return std::span(buffer).first(length);
    }
    std::array<hal::byte, 23> buffer{};
    std::size_t length = 0;
  };

  class ch9329_parameters
  {
  public:
    ch9329_parameters(std::array<hal::byte, 50> p_config_bytes);
    hal::byte set_parameters(ch9329_parameters p_parameters);
    void set_config_bytes(std::array<hal::byte, 50> p_config_bytes)
    {
      m_config_bytes = p_config_bytes;
    };
    auto const& get_config_bytes() const
    {
      return m_config_bytes;
    }
    hal::byte chip_working_mode;
    hal::byte serial_communication_mode;
    hal::byte serial_address;
    std::uint32_t serial_mode_baud_rate;
    std::uint16_t serial_mode_packet_interval;
    std::uint16_t vendor_id;
    std::uint16_t p_id;
    std::uint16_t ascii_mode_kb_upload_interval;
    std::uint16_t ascii_mode_kb_release_delay;
    hal::byte ascii_mode_kb_auto_enter;
    std::uint32_t ascii_mode_kb_carriage_return_1;
    std::uint32_t ascii_mode_kb_carriage_return_2;
    std::uint32_t kb_start_filter_chars;
    std::uint32_t kb_end_filter_chars;
    hal::byte usb_string_enable;
    hal::byte ascii_mode_kb_fast_upload_mode;

  private:
    std::array<hal::byte, 50> m_config_bytes = {};
  };

  /**
   * @brief Holds data and functions related to using mouse absolute position
   * commands
   *
   */
  class mouse_absolute
  {
  public:
    /**
     * @brief Construct a new mouse absolute object
     *
     * @param p_screen_width width of the screen, used as max X value
     * @param p_screen_height height of the screen, used as max Y value
     */
    mouse_absolute(std::uint16_t p_screen_width, std::uint16_t p_screen_height);
    /**
     * @brief Change the position of the cursor to specified x, y coordinates
     *
     * @param p_x X coordinate to move to
     * @param p_y Y coordinate to move to
     * @return mouse_absolute&
     */
    mouse_absolute& position(std::uint16_t p_x, std::uint16_t p_y);
    /**
     * @brief Change the position of the scroll wheel. Negative values indicates
     * scrolling down and positive indicates scrolling up
     *
     * @param p_scroll_offset number of "scroll teeth" to move
     * @return mouse_absolute&
     */
    mouse_absolute& scroll_position(std::int8_t p_scroll_offset);
    /**
     * @brief Change the state of the left button
     *
     * @param p_pressed State of button, true means pressed and false is
     * released
     * @return mouse_absolute&
     */
    mouse_absolute& left_button(bool p_pressed);
    /**
     * @brief Change the state of the middle button
     *
     * @param p_pressed State of button, true means pressed and false is
     * released
     * @return mouse_absolute&
     */
    mouse_absolute& middle_button(bool p_pressed);
    /**
     * @brief Change the state of the right button
     *
     * @param p_pressed State of button, true means pressed and false is
     * released
     * @return mouse_absolute&
     */
    mouse_absolute& right_button(bool p_pressed);
    /**
     * @brief Get the screen width
     *
     * @return std::uint16_t Screen width that was set in contructor
     */
    std::uint16_t get_screen_width()
    {
      return m_screen_width;
    };
    /**
     * @brief Get the screen height
     *
     * @return std::uint16_t Screen height that was set in contructor
     */
    std::uint16_t get_screen_height()
    {
      return m_screen_height;
    };
    /**
     * @brief Get the data array containing the control bytes
     *
     * @return auto const& Byte array containing control information
     */
    auto const& get_data() const
    {
      return m_data;
    }

  private:
    std::uint16_t m_screen_width;
    std::uint16_t m_screen_height;
    std::array<hal::byte, 7> m_data = {};
  };

  /**
   * @brief Holds data and functions related to using mouse relative position
   * commands
   *
   */
  class mouse_relative
  {
  public:
    /**
     * @brief Construct a new mouse relative object
     *
     */
    mouse_relative();
    /**
     * @brief Move the cursor relative to where it currently is.
     *
     * @param p_x_offset Number of pixels to move the cursor on the x axis.
     * Negative values move left and positive move right.
     * @param p_y_offset Number of pixels to move the cursor on the y axis.
     * Negative values move down and positive move up.
     * @return mouse_relative&
     */
    mouse_relative& move(std::int8_t p_x_offset, std::int8_t p_y_offset);
    /**
     * @brief Change the position of the scroll wheel. Negative values indicates
     * scrolling down and positive indicates scrolling up
     *
     * @param p_scroll_offset number of "scroll teeth" to move
     * @return mouse_relative&
     */
    mouse_relative& scroll_position(std::int8_t p_scroll_offset);
    /**
     * @brief Change the state of the left button
     *
     * @param p_pressed State of button, true means pressed and false is
     * released
     * @return mouse_relative&
     */
    mouse_relative& left_button(bool p_pressed);
    /**
     * @brief Change the state of the middle button
     *
     * @param p_pressed State of button, true means pressed and false is
     * released
     * @return mouse_relative&
     */
    mouse_relative& middle_button(bool p_pressed);
    /**
     * @brief Change the state of the right button
     *
     * @param p_pressed State of button, true means pressed and false is
     * released
     * @return mouse_relative&
     */
    mouse_relative& right_button(bool p_pressed);
    /**
     * @brief Get the data array containing the control bytes
     *
     * @return auto const& Byte array containing control information
     */
    auto const& get_data() const
    {
      return m_data;
    }

  private:
    std::array<hal::byte, 5> m_data = {};
  };

  /**
   * @brief Holds data and functions related to using keyboard media commands
   */
  class keyboard_media
  {
  public:
    /**
     * @brief Construct a new keyboard media object
     *
     */
    keyboard_media();
    /**
     * @brief Press a media key.
     *
     * @param p_key media_key enum representing which key to press
     * @return keyboard_media&
     */
    keyboard_media& press_media_key(media_key p_key);
    /**
     * @brief Release a media key.
     *
     * @param p_key media_key enum representing which key to release
     * @return keyboard_media&
     */
    keyboard_media& release_media_key(media_key p_key);
    /**
     * @brief Release all acpi keys
     *
     * @return keyboard_media&
     */
    keyboard_media& release_all_keys();
    auto const& get_data() const
    {
      return m_data;
    }

  private:
    std::array<hal::byte, 4> m_data = {};
  };

  /**
   * @brief Holds data and functions related to using keyboard acpi commands
   */
  class keyboard_acpi
  {
  public:
    /**
     * @brief Construct a new keyboard acpi object
     *
     */
    keyboard_acpi();
    /**
     * @brief Press an acpi key.
     *
     * @param p_key acpi_key enum representing which key to release
     * @return keyboard_acpi&
     */
    keyboard_acpi& press_acpi_key(acpi_key p_key);
    /**
     * @brief Release an acpi key.
     *
     * @param p_key acpi_key enum representing which key to release
     * @return keyboard_acpi&
     */
    keyboard_acpi& release_acpi_key(acpi_key p_key);
    /**
     * @brief Release all acpi keys
     *
     * @return keyboard_acpi&
     */
    keyboard_acpi& release_all_keys();
    /**
     * @brief Get the data array containing the control bytes for acpi keys
     *
     * @return auto const& Byte array containing control information
     */
    auto const& get_data() const
    {
      return m_data;
    }

  private:
    std::array<hal::byte, 2> m_data = {};
  };

  /**
   * @brief Holds data and functions related to using keyboard general commands
   *
   */
  class keyboard_general
  {
  public:
    /**
     * @brief Construct a new keyboard general object
     *
     */
    keyboard_general();
    /**
     * @brief Press a control key.
     *
     * Control keys are left and right control, shift, alt, and windows keys.
     *
     * @param p_key control_key_bit enum value representing which key to press
     * @return keyboard_general&
     */
    keyboard_general& press_control_key(control_key_bit p_key);
    /**
     * @brief Press a normal key.
     *
     * @param p_key normal_key enum value representing which key to press
     * @param p_slot Which slot number to put the key into. Valid values are 1 -
     * 6
     * @return keyboard_general&
     */
    keyboard_general& press_normal_key(normal_key p_key, uint8_t p_slot);
    /**
     * @brief Release a control key.
     *
     * @param p_key control_key_bit enum value representing which key to release
     * @return keyboard_general&
     */
    keyboard_general& release_control_key(control_key_bit p_key);
    /**
     * @brief Release a normal key.
     *
     * @param p_key normal_key enum value representing which key to press
     * @return keyboard_general&
     */
    keyboard_general& release_normal_key(normal_key p_key);
    /**
     * @brief Release all keys, this includes control and normal keys.
     *
     * @return keyboard_general&
     */
    keyboard_general& release_all_keys();
    /**
     * @brief Get the data array containing the control bytes
     *
     * @return auto const& Byte array containing control information
     */
    auto const& get_data() const
    {
      return m_data;
    }

  private:
    std::array<hal::byte, 8> m_data = {};
  };

  /**
   * @brief Construct a new ch9329 object
   *
   * @param p_uart uart used to communicate with CH9329
   */
  ch9329(hal::serial& p_uart);
  /**
   * @brief Send mouse absolute position command
   *
   * @param p_data mouse absolute object containing command bytes
   */
  void send(mouse_absolute const& p_data);
  /**
   * @brief Send mouse relative position command
   *
   * @param p_data mouse relative object containing command bytes
   */
  void send(mouse_relative const& p_data);
  /**
   * @brief Send keyboard media command
   *
   * @param p_data keyboard media object containing command bytes
   */
  void send(keyboard_media const& p_data);
  /**
   * @brief Send keyboard acpi command
   *
   * @param p_data
   */
  void send(keyboard_acpi const& p_data);
  /**
   * @brief Send keyboard general command
   *
   * @param p_data keyboard general object containing command bytes
   */
  void send(keyboard_general const& p_data);
  hal::byte set_parameters(ch9329_parameters const& p_data);
  ch9329_parameters get_parameters();

  chip_info get_info();
  usb_string_descriptor get_manufacturer_descriptor();
  usb_string_descriptor get_product_descriptor();
  usb_string_descriptor get_serial_number_descriptor();
  hal::byte set_manufacturer_descriptor(std::string_view p_string);
  hal::byte set_product_descriptor(std::string_view p_string);
  hal::byte set_serial_number_descriptor(std::string_view p_string);
  hal::byte restore_factory_default_settings();
  hal::byte reset();

private:
  hal::serial* m_uart;
};
}  // namespace hal::input
