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

#include <boost/ut.hpp>

namespace hal::input {
boost::ut::suite<"ch9329_test"> ch9329_test = [] {
  using namespace boost::ut;
  using namespace std::literals;

  static constexpr hal::byte write_failure_byte{ 'C' };
  static constexpr hal::byte filler_byte{ 'A' };

  class fake_serial : public hal::serial
  {
  public:
    void driver_configure(settings const&) override
    {
    }

    write_t driver_write(std::span<hal::byte const> p_data) override
    {
      m_write_call_count++;
      if (p_data[0] == write_failure_byte) {
        safe_throw(hal::io_error(this));
      }
      m_out = p_data;

      if (m_single_byte_out) {
        return write_t{ p_data.subspan(0, 1) };
      }
      return write_t{ p_data };
    }

    read_t driver_read(std::span<hal::byte> p_data) override
    {
      if (p_data.size() == 0) {
        return read_t{
          .data = p_data,
          .available = 1,
          .capacity = 1,
        };
      }

      m_read_was_called = true;

      if (m_read_fails) {
        safe_throw(hal::io_error(this));
      }

      // only fill 1 byte at a time
      p_data[0] = filler_byte;

      return read_t{
        .data = p_data.subspan(0, 1),
        .available = 1,
        .capacity = 1,
      };
    }

    void driver_flush() override
    {
      m_flush_called = true;
    }

    ~fake_serial() override = default;

    std::span<hal::byte const> m_out{};
    int m_write_call_count = 0;
    bool m_read_was_called = false;
    bool m_flush_called = false;
    bool m_read_fails = false;
    bool m_single_byte_out = false;
  };

  "ch9329::ch9329()"_test = []() {
    // Setup
    fake_serial test_serial;
    hal::input::ch9329 usb_control(test_serial);
    // Exercise
    // Verify
  };
};
}  // namespace hal::input
