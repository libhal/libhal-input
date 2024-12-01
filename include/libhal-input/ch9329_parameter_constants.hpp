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

enum class working_mode : hal::byte
{
  // software mode keyboard (normal & multimedia) and mouse
  software_keyboard_and_mouse = 0x00,
  // software mode keyboard (normal) only
  software_keyboard_only = 0x01,
  // software mode mouse only
  software_mouse_only = 0x02,
  // software mode HID mode
  software_hid_mode = 0x03,
  // hardware mode keyboard (normal & multimedia) and mouse
  hardware_keyboard_and_mouse = 0x80,
  // hardware mode keyboard (normal) only
  hardware_keyboard_only = 0x81,
  // hardware mode mouse only
  hardware_mouse_only = 0x82,
  // hardware mode HID mode
  hardware_hid_mode = 0x83
};

enum class communication_mode : hal::byte
{
  // software set protocol transmission mode
  software_protocol_mode = 0x00,
  // software set ascii mode
  software_ascii_mode = 0x01,
  // software set transparent mode
  software_transparent_mode = 0x02,
  // hardware set protocol transmission mode
  hardware_protocol_mode = 0x80,
  // hardware set ascii mode
  hardware_ascii_mode = 0x81,
  // hardware set transparent mode
  hardware_transparent_mode = 0x82
};

enum class custom_descriptor : std::uint8_t
{
  serial_number = 0,
  product_string = 1,
  vendor_string = 2,
  usb_string = 7
};

enum class lock_status : std::uint8_t
{
  num_lock = 0,
  caps_lock = 1,
  scroll_lock = 2
};
