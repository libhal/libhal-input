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

#include <cstdint>

#include <libhal-input/ch9329.hpp>
#include <libhal-input/gamepad/nunchuck.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/i2c.hpp>

#include <resource_list.hpp>

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock.value();
  auto& console = *p_map.console.value();
  auto& uart3 = *p_map.uart3.value();

  hal::input::ch9329 usb_control(uart3);
  uint16_t width = 3840;
  uint16_t height = 2160;

  hal::input::ch9329::mouse_absolute abs_mouse_control(width, height);

  hal::print(console, "Demo Application Starting...\n\n");
  hal::print<32>(console, "Screen Width: %li ", width);
  hal::print<32>(console, "Screen Height: %li \n", height);

  while (true) {
    // top left corner
    abs_mouse_control.position(1, 1);
    usb_control.send(abs_mouse_control);
    hal::delay(clock, 1s);

    // middle screen
    abs_mouse_control.position(width / 2, height / 2);
    usb_control.send(abs_mouse_control);
    hal::delay(clock, 1s);

    // bottom right
    abs_mouse_control.position(width, height);
    usb_control.send(abs_mouse_control);
    hal::delay(clock, 1s);

    // bottom left
    abs_mouse_control.position(1, height);
    usb_control.send(abs_mouse_control);
    hal::delay(clock, 1s);

    // middle screen
    abs_mouse_control.position(width / 2, height / 2);
    usb_control.send(abs_mouse_control);
    hal::delay(clock, 1s);

    // top right
    abs_mouse_control.position(width, 1);
    usb_control.send(abs_mouse_control);
    hal::delay(clock, 1s);
  }
}
