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

#include <cstdint>

#include <libhal/units.hpp>

enum class control_key : std::uint8_t
{
  left_control = 0,
  left_shift = 1,
  left_alt = 2,
  left_windows = 3,
  right_control = 4,
  right_shift = 5,
  right_alt = 6,
  right_windows = 7,
};

enum class acpi_key : std::uint8_t
{
  power = 0,
  sleep = 1,
  wake_up = 2
};

enum class media_key : std::uint8_t
{
  volume_up = 0,
  volume_down = 1,
  mute = 2,
  play_pause = 3,
  next_track = 4,
  prev_track = 5,
  cd_stop = 6,
  eject = 7,
  email = 8,
  internet_search = 9,
  internet_favorites = 10,
  internet_home = 11,
  internet_back = 12,
  internet_forward = 13,
  internet_stop = 14,
  refresh = 15,
  media = 16,
  explorer = 17,
  calculator = 18,
  screen_save = 19,
  my_computer = 20,
  minimize = 21,
  record = 22,
  rewind = 23
};

enum class normal_key : hal::byte
{
  a = 0x04,
  b = 0x05,
  c = 0x06,
  d = 0x07,
  e = 0x08,
  f = 0x09,
  g = 0x0A,
  h = 0x0B,
  i = 0x0C,
  j = 0x0D,
  k = 0x0E,
  l = 0x0F,
  m = 0x10,
  n = 0x11,
  o = 0x12,
  p = 0x13,
  q = 0x14,
  r = 0x15,
  s = 0x16,
  t = 0x17,
  u = 0x18,
  v = 0x19,
  w = 0x1A,
  x = 0x1B,
  y = 0x1C,
  z = 0x1D,
  k1 = 0x1E,
  k2 = 0x1F,
  k3 = 0x20,
  k4 = 0x21,
  k5 = 0x22,
  k6 = 0x23,
  k7 = 0x24,
  k8 = 0x25,
  k9 = 0x26,
  k0 = 0x27,
  left_enter = 0x28,
  esc = 0x29,
  back_space = 0x2A,
  tab = 0x2B,
  space = 0x2C,
  hyphen = 0x2D,
  equal = 0x2E,
  left_bracket = 0x2F,
  right_bracket = 0x30,
  keycode_29 = 0x31,
  keycode_42 = 0x32,
  semi_colon = 0x33,
  quote_mark = 0x34,
  grave = 0x35,
  comma = 0x36,  // this might be < only? datasheet is weird
  period = 0x37,
  slash = 0x38,
  capslock = 0x39,
  f1 = 0x3A,
  f2 = 0x3B,
  f3 = 0x3C,
  f4 = 0x3D,
  f5 = 0x3E,
  f6 = 0x3F,
  f7 = 0x40,
  f8 = 0x41,
  f9 = 0x42,
  f10 = 0x43,
  f11 = 0x44,
  f12 = 0x45,
  print_screen = 0x46,
  scroll_lock = 0x47,
  pause = 0x48,
  insert = 0x49,
  home = 0x4A,
  pg_up = 0x4B,
  delete_key = 0x4C,
  end = 0x4D,
  pg_dn = 0x4E,
  right_arrow = 0x4F,
  left_arrow = 0x50,
  down_arrow = 0x51,
  up_arrow = 0x52,
  num_lock = 0x53,
  num_pad_asterisk = 0x55,
  num_pad_minus = 0x56,
  num_pad_plus = 0x57,
  num_pad_enter = 0x58,
  num_pad_1 = 0x59,
  num_pad_2 = 0x5A,
  num_pad_3 = 0x5B,
  num_pad_4 = 0x5C,
  num_pad_5 = 0x5D,
  num_pad_6 = 0x5E,
  num_pad_7 = 0x5F,
  num_pad_8 = 0x60,
  num_pad_9 = 0x61,
  num_pad_0 = 0x62,
  num_pad_del = 0x63,
  keycode_45 = 0x64,
  keycode_107 = 0x85,
  keycode_14 = 0x89,
  keycode_56 = 0x87,
  left_ctrl = 0xE0,
  left_shift = 0xE1,
  left_alt = 0xE2,
  right_ctrl = 0xE4,
  right_shift = 0xE5,
  right_alt = 0xE6,
};
