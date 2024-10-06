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

#include <libhal-input/ch9329_keyboard_constants.hpp>
#include <libhal/serial.hpp>
#include <libhal/units.hpp>

namespace hal::input {
/**
 * @brief Driver for CH9329 UART to USB
 *
 */
constexpr auto mouse_abs_data_size = 7;
constexpr auto mouse_rel_data_size = 5;
constexpr auto kb_media_data_size = 4;
constexpr auto kb_acpi_data_size = 2;
constexpr auto kb_general_data_size = 8;

class ch9329
{
public:
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
    std::array<hal::byte, mouse_abs_data_size> m_data = {};
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
    std::array<hal::byte, mouse_rel_data_size> m_data = {};
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
    std::array<hal::byte, kb_media_data_size> m_data = {};
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
    std::array<hal::byte, kb_acpi_data_size> m_data = {};
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
    std::array<hal::byte, kb_general_data_size> m_data = {};
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
   * @brief Reset the chip using the cmd_reset command
   *
   * @return hal::byte status byte returned from the chip
   */
  hal::byte reset();

private:
  hal::serial* m_uart;
};
}  // namespace hal::input
