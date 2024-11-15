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

namespace andino {

struct Hw {
  /// @brief Left encoder channel A pin. Connected to 15 (GPIO 15).
  static const unsigned int kLeftEncoderChannelAGpioPin{15};
  /// @brief Left encoder channel B pin. Connected to 2 (GPIO 2).
  static const unsigned int kLeftEncoderChannelBGpioPin{2};

  /// @brief Right encoder channel A pin. Connected to 12 (GPIO 12).
  static const unsigned int kRightEncoderChannelAGpioPin{12};
  /// @brief Right encoder channel B pin. Connected to 14 (GPIO 14).
  static const unsigned int kRightEncoderChannelBGpioPin{14};

  /// @brief Left motor driver backward pin. Connected to 19 (GPIO 19).
  static const unsigned int kLeftMotorBackwardGpioPin{19};
  /// @brief Left motor driver forward pin. Connected to 18 (GPIO 18).
  static const unsigned int kLeftMotorForwardGpioPin{18};

  /// @brief Right motor driver backward pin. Connected to 22 (GPIO 22).
  static const unsigned int kRightMotorBackwardGpioPin{22};
  /// @brief Right motor driver forward pin. Connected to 23 (GPIO 23).
  static const unsigned int kRightMotorForwardGpioPin{23};

  /// @brief Left motor driver enable pin. Connected to 32 (GPIO 32).
  static const unsigned int kLeftMotorEnableGpioPin{32};
  /// @brief Right motor driver enable pin. Connected to 33 (GPIO 33).
  static const unsigned int kRightMotorEnableGpioPin{33};
};

}  // namespace andino
