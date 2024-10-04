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

  constexpr auto sensitivity = 16;
  auto& clock = *p_map.clock.value();
  auto& console = *p_map.console.value();
  auto& uart3 = *p_map.uart3.value();
  auto& i2c = *p_map.i2c.value();

  hal::input::nunchuck nunchuck(i2c);
  hal::input::ch9329 usb_control(uart3);
  hal::input::ch9329::keyboard_general kb_control;
  hal::input::ch9329::mouse_relative rel_mouse_control;

  hal::print(console, "Demo Application Starting...\n\n");
  auto params = usb_control.get_parameters();
  hal::print<32>(
    console, "Chip working mode: %0X \n", params.get_chip_working_mode());
  hal::print<32>(console,
                 "Communication mode: %0X \n",
                 params.get_serial_communication_mode());
  hal::print<32>(console, "Baud: %lu \n", params.get_serial_mode_baud_rate());
  params.set_serial_mode_baud_rate(115200);
  auto response = usb_control.set_parameters(params);
  params = usb_control.get_parameters();
  hal::print<32>(console, "set params response: %0X \n", response);
  hal::print<32>(console, "Baud: %lu \n", params.get_serial_mode_baud_rate());

  hal::print(console, "Loop Starting...\n\n");
  bool btn_prev_state = false;
  while (true) {
    auto data = nunchuck.read();
    std::int8_t x = (data.joystick_x() - 128) / sensitivity;
    std::int8_t y = -(data.joystick_y() - 128) / sensitivity;
    rel_mouse_control.move(x, y).left_button(data.c_button());
    usb_control.send(rel_mouse_control);

    // only send keyboard commands if btn state has changed
    bool z_button_state = data.z_button();
    if (z_button_state && z_button_state != btn_prev_state) {
      btn_prev_state = z_button_state;
      kb_control.press_normal_key(normal_key::a, 1);
      usb_control.send(kb_control);
    }
    if (!z_button_state && z_button_state != btn_prev_state) {
      btn_prev_state = z_button_state;
      kb_control.release_normal_key(normal_key::a);
      usb_control.send(kb_control);
    }
    hal::delay(clock, 1ms);
  }
}
