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

#include <cstdint>
#include <libhal-input/ch9329_kb_bytes.hpp>
#include <libhal/serial.hpp>
#include <libhal/units.hpp>

namespace hal::input {
/**
 * @brief Driver for CH9329 UART to USB
 *
 */
class ch9329
{
public:
/**
 * @brief Holds information about the chip from the cmd_get_info command 
 * 
 */
  struct chip_info
  {
    /// chip version, 0x30 indicates version 1.0
    hal::byte version;
    /// usb enumeration status of the chip
    bool enumeration_status;
    /// status of num lock (0 means off, 1 is on)
    bool num_lock;
    /// status of caps lock (0 means off, 1 is on)
    bool caps_lock;
    /// status of scroll lock (0 means off, 1 is on)
    bool scroll_lock;
  };

/**
 * @brief Holds data used to get and set usb string descriptors
 * 
 */
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

  /**
  * @brief Holds data and functions related to getting and setting parameter 
  * configurations on the chip
  * 
  */
  class ch9329_parameters
  {
  public:
    /**
     * @brief Construct a new ch9329 parameters object with all 0x00 data bytes
     * 
     */
    ch9329_parameters();
    /**
     * @brief Construct a new ch9329 parameters object with an array of bytes 
     * containing parameters
     * 
     * @param p_config_bytes array holding parameter settings in bytes
     */
    ch9329_parameters(std::array<hal::byte, 50> p_config_bytes);
    
    /**
     * @brief Set the array containing parameter bytes
     * 
     * @param p_config_bytes bytes to use
     */
    void set_config_bytes(std::array<hal::byte, 50> p_config_bytes)
    {
      m_config_bytes = p_config_bytes;
    };
    /**
     * @brief Set the chip working mode.
     * 
     * @param p_working_mode 
     */
    void set_chip_working_mode(hal::byte p_working_mode);
    /**
     * @brief Set the serial communication mode
     * 
     * @param p_communication_mode 
     */
    void set_serial_communication_mode(hal::byte p_communication_mode);
    /**
     * @brief Set the serial mode baud rate
     * 
     * @param p_baud 
     */
    void set_serial_mode_baud_rate(std::uint32_t p_baud);
    /**
     * @brief Set the serial address
     * 
     * @param p_serial 
     */
    void set_serial_address(hal::byte p_serial);
    /**
     * @brief Set the serial mode packet interval
     * 
     * @param p_packet_interval 
     */
    void set_serial_mode_packet_interval(std::uint16_t p_packet_interval);
    /**
     * @brief Set the vendor id
     * 
     * @param p_vid 
     */
    void set_vendor_id(std::uint16_t p_vid);
    /**
     * @brief Set the product id
     * 
     * @param p_pid 
     */
    void set_product_id(std::uint16_t p_pid);
    /**
     * @brief Set the ascii mode keyboard upload interval
     * 
     * @param p_upload_interval 
     */
    void set_ascii_mode_kb_upload_interval(std::uint16_t p_upload_interval);
    /**
     * @brief Set the ascii mode keyboard release delay
     * 
     * @param p_release_dealy 
     */
    void set_ascii_mode_kb_release_delay(std::uint16_t p_release_dealy);
    /**
     * @brief Set the ascii mode keyboard auto enter
     * 
     * @param p_auto_enter 
     */
    void set_ascii_mode_kb_auto_enter(hal::byte p_auto_enter);
    /**
     * @brief Set the ascii mode keyboard carriage return 1
     * 
     * @param p_carriage_return 
     */
    void set_ascii_mode_kb_carriage_return_1(std::uint32_t p_carriage_return);
    /**
    * @brief Set the ascii mode keyboard carriage return 2
    * 
    * @param p_carriage_return 
    */
    void set_ascii_mode_kb_carriage_return_2(std::uint32_t p_carriage_return);
    /**
     * @brief Set the keyboard start filter characters
     * 
     * @param p_start_filters 
     */
    void set_kb_start_filter_chars(std::uint32_t p_start_filters);
    /**
     * @brief Set the keyboard end filter characters
     * 
     * @param p_end_filters 
     */
    void set_kb_end_filter_chars(std::uint32_t p_end_filters);
    /**
     * @brief Set the usb string enable flag
     * 
     * @param p_enable 
     */
    void set_usb_string_enable(hal::byte p_enable);
    /**
     * @brief Set the ascii mode keyboard fast upload mode
     * 
     * @param p_fast_upload 
     */
    void set_ascii_mode_kb_fast_upload_mode(hal::byte p_fast_upload);

    /**
     * @brief Get the chip working mode
     * 
     * @return hal::byte 
     */
    hal::byte get_chip_working_mode();
    /**
     * @brief Get the serial communication mode
     * 
     * @return hal::byte 
     */
    hal::byte get_serial_communication_mode();
    /**
     * @brief Get the serial address
     * 
     * @return hal::byte 
     */
    hal::byte get_serial_address();
    /**
     * @brief Get the serial mode baud rate
     * 
     * @return std::uint32_t 
     */
    std::uint32_t get_serial_mode_baud_rate();
    /**
     * @brief Get the serial mode packet interval
     * 
     * @return std::uint16_t 
     */
    std::uint16_t get_serial_mode_packet_interval();
    /**
     * @brief Get the vendor id
     * 
     * @return std::uint16_t 
     */
    std::uint16_t get_vendor_id();
    /**
     * @brief Get the product id
     * 
     * @return std::uint16_t 
     */
    std::uint16_t get_product_id();
    /**
     * @brief Get the ascii mode keyboard upload interval
     * 
     * @return std::uint16_t 
     */
    std::uint16_t get_ascii_mode_kb_upload_interval();
    /**
     * @brief Get the ascii mode keyboard release delay
     * 
     * @return std::uint16_t 
     */
    std::uint16_t get_ascii_mode_kb_release_delay();
    /**
     * @brief Get the ascii mode keyboard auto enter flag
     * 
     * @return hal::byte 
     */
    hal::byte get_ascii_mode_kb_auto_enter();
    /**
     * @brief Get the ascii mode keyboard carriage return 1
     * 
     * @return std::uint32_t 
     */
    std::uint32_t get_ascii_mode_kb_carriage_return_1();
    /**
     * @brief Get the ascii mode keyboard carriage return 2
     * 
     * @return std::uint32_t 
     */
    std::uint32_t get_ascii_mode_kb_carriage_return_2();
    /**
     * @brief Get the keyboard start filter characters
     * 
     * @return std::uint32_t 
     */
    std::uint32_t get_kb_start_filter_chars();
    /**
     * @brief Get the keyboard end filter characters
     * 
     * @return std::uint32_t 
     */
    std::uint32_t get_kb_end_filter_chars();
    /**
     * @brief Get the usb string enable flag
     * 
     * @return hal::byte 
     */
    hal::byte get_usb_string_enable();
    /**
     * @brief Get the ascii mode keyboard fast upload mode
     * 
     * @return hal::byte 
     */
    hal::byte get_ascii_mode_kb_fast_upload_mode();
    /**
     * @brief Get the config bytes array
     * 
     * @return auto const& 
     */
    auto const& get_config_bytes() const
    {
      return m_config_bytes;
    }

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

  /**
   * @brief Set the parameters configuration of the chip
   * 
   * @param p_data object containing parameters to change
   * @return hal::byte status byte returned from chip
   */
  hal::byte set_parameters(ch9329_parameters const& p_data);
  /**
   * @brief Get the current parameters configuration from the chip
   * 
   * @return ch9329_parameters contains parameters and the response bytes
   */
  ch9329_parameters get_parameters();
/**
 * @brief Get info about the chip
 * 
 * @return chip_info contains basic information about the chip
 */
  chip_info get_info();
  /**
   * @brief Get the manufacturer usb string descriptor on the chip
   * 
   * @return usb_string_descriptor contains buffer and trimmed data
   */
  usb_string_descriptor get_manufacturer_descriptor();
  /**
   * @brief Get the product usb string descriptor on the chip
   * 
   * @return usb_string_descriptor contains buffer and trimmed data
   */
  usb_string_descriptor get_product_descriptor();
  /**
   * @brief Get the serial number usb string descriptor on the chip
   * 
   * @return usb_string_descriptor contains buffer and trimmed data
   */
  usb_string_descriptor get_serial_number_descriptor();
  /**
   * @brief Set the manufacturer usb string descriptor
   * 
   * @param p_string string to use as descriptor
   * @return hal::byte status byte returned from the chip
   */
  hal::byte set_manufacturer_descriptor(std::string_view p_string);
  /**
   * @brief Set the product usb string descriptor
   * 
   * @param p_string string to use as descriptor
   * @return hal::byte status byte returned from the chip
   */
  hal::byte set_product_descriptor(std::string_view p_string);
  /**
   * @brief Set the serial number usb string descriptor
   * 
   * @param p_string string to use as descriptor
   * @return hal::byte status byte returned from the chip
   */
  hal::byte set_serial_number_descriptor(std::string_view p_string);
  /**
   * @brief Restore all settings to factory default
   * 
   * @return hal::byte status byte returned from the chip
   */
  hal::byte restore_factory_default_settings();
  /**
   * @brief Reset the chip using the cmd_reset command
   * 
   * @return hal::byte status byte returned from the chip
   */
  hal::byte reset();

private:
  hal::serial* m_uart;
};
}  // namespace hal::input
