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
#include <libhal/serial.hpp>
#include <libhal/units.hpp>

namespace hal::input {
/**
 * @brief Driver for CH9329 UART to USB
 *
 */
class ch9329
{
  /**
   * @brief Holds data and functions related to using mouse absolute position
   * commands
   *
   */
public:
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

private:
  hal::serial* m_uart;
};
}  // namespace hal::input
