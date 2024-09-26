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

  hal::input::ch9329::keyboard_general keyboard_control;

  hal::print(console, "Demo Application Starting...\n\n");

  while (true) {
    keyboard_control.press_normal_key(normal_key::a, 1);
    usb_control.send(keyboard_control);
    hal::delay(clock, 1ms);

    keyboard_control.release_normal_key(normal_key::a);
    usb_control.send(keyboard_control);
    hal::delay(clock, 2s);

    keyboard_control.press_normal_key(normal_key::back_space, 1);
    usb_control.send(keyboard_control);
    hal::delay(clock, 1ms);

    keyboard_control.release_normal_key(normal_key::back_space);
    usb_control.send(keyboard_control);
    hal::delay(clock, 2s);
  }
}
