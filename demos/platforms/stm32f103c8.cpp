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

#include <libhal/units.hpp>

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>

#include <libhal-soft/bit_bang_i2c.hpp>
#include <libhal-stm32f1/clock.hpp>
#include <libhal-stm32f1/constants.hpp>
#include <libhal-stm32f1/output_pin.hpp>
#include <libhal-stm32f1/uart.hpp>

#include <resource_list.hpp>

resource_list initialize_platform()
{
  using namespace hal::literals;

  // Set the MCU to the maximum clock speed
  hal::stm32f1::maximum_speed_using_internal_oscillator();

  static hal::cortex_m::dwt_counter counter(
    hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu));

  static hal::stm32f1::uart uart1(hal::port<1>,
                                  hal::buffer<128>,
                                  hal::serial::settings{
                                    .baud_rate = 115200,
                                  });
  static hal::stm32f1::uart uart3(hal::port<3>,
                                  hal::buffer<128>,
                                  hal::serial::settings{
                                    .baud_rate = 9600,
                                  });

  static hal::stm32f1::output_pin led('C', 13);
  static hal::stm32f1::output_pin sda('B', 7);
  static hal::stm32f1::output_pin scl('B', 6);
  // TODO(#4): Remove configure calls
  sda.configure({ .resistor = hal::pin_resistor::pull_up, .open_drain = true });
  scl.configure({ .resistor = hal::pin_resistor::pull_up, .open_drain = true });
  static hal::bit_bang_i2c::pins pins{ .sda = &sda, .scl = &scl };
  static hal::bit_bang_i2c bit_bang_i2c(pins, counter);
  bit_bang_i2c.configure(hal::i2c::settings{ .clock_rate = 100.0_kHz });

  return {
    .reset = +[]() { hal::cortex_m::reset(); },
    .console = &uart1,
    .uart3 = &uart3,
    .clock = &counter,
    .status_led = &led,
    .i2c = &bit_bang_i2c,
  };
}
