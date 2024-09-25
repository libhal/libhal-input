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
class ch9329
{

public:
  class mouse_absolute
  {
  public:
    mouse_absolute(std::uint16_t p_screen_width, std::uint16_t p_screen_height);
    mouse_absolute& position(std::uint16_t p_x, std::uint16_t p_y);
    mouse_absolute& scroll_position(std::int8_t p_scroll_offset);
    mouse_absolute& left_button(bool p_pressed);
    mouse_absolute& middle_button(bool p_pressed);
    mouse_absolute& right_button(bool p_pressed);
    std::uint16_t get_screen_width();
    std::uint16_t get_screen_height();
    auto const& get_data() const
    {
      return m_data;
    }

  private:
    std::uint16_t m_screen_width;
    std::uint16_t m_screen_height;
    std::array<hal::byte, 7> m_data = {};
  };

  class mouse_relative
  {
  public:
    mouse_relative();
    mouse_relative& move(std::int8_t p_x_offset, std::int8_t p_y_offset);
    mouse_relative& scroll_position(std::int8_t p_scroll_offset);
    mouse_relative& left_button(bool p_pressed);
    mouse_relative& middle_button(bool p_pressed);
    mouse_relative& right_button(bool p_pressed);
    auto const& get_data() const
    {
      return m_data;
    }

  private:
    std::array<hal::byte, 5> m_data = {};
  };

  ch9329(hal::serial& p_uart);

  void send(mouse_absolute const& p_data);
  void send(mouse_relative const& p_data);

private:
  hal::serial* m_uart;
};
}  // namespace hal::input
