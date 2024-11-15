// BSD 3-Clause License
//
// Copyright (c) 2024, Ekumen Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#pragma once
#include <Arduino.h>

namespace andino {

/// @brief This class allows to use a quadrature encoder by configuring it and
/// then getting its ticks count value.
class EncoderDriver {
 public:
  /// @brief Constructs a new Encoder object.
  ///
  /// @param encoder_channel_a Digital interrupt input connected to encoder
  /// channel A pin.
  /// @param encoder_channel_b Digital interrupt input connected to encoder
  /// channel B pin.
  EncoderDriver(const int& encoder_channel_a, const int& encoder_channel_b);

  /// @brief Initializes the encoder.
  void begin();

  /// @brief Channels interrupt callback.
  void IRAM_ATTR callback();

  /// @brief Gets the ticks count value.
  ///
  /// @return Ticks count value.
  int read();

 private:
  /// Digital interrupt input connected to encoder channel A pin.
  int encoder_channel_a_{0};
  /// Digital interrupt input connected to encoder channel B pin.
  int encoder_channel_b_{0};
  /// Number of constructed Encoder instances.
  volatile int instance_count_{0};

  /// Previous State of channel A and channel B encoder.
  volatile byte previous_state_{0};
  /// Current State of channel A and channel B encoder.
  volatile byte current_state_{0};
};
}  // namespace andino
