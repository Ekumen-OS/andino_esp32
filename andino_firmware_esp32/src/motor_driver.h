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

/// @brief This class allows to control a DC motor by enabling it and setting
/// its speed. The involved pins are expected to be connected to a full-bridge
/// motor driver module, such as the L298N.
class MotorDriver {
 public:
  /// @brief Constructs a new MotorDriver object.
  ///
  /// @param enable_pwm_out PWM output connected to motor enable pin.
  /// @param forward_out Output connected to motor forward pin.
  /// @param backward_out Output connected to motor backward pin.
  MotorDriver(const int& enable_pwm_out, const int& forward_out, const int& backward_out);

  /// @brief Initializes the motor.
  void begin();

  /// @brief Sets the motor speed.
  ///
  /// @param speed Motor speed value.
  void set_speed(int speed);

 private:
  /// Minimum speed value (negative speeds are considered as positive backward
  /// speeds).
  static constexpr int kMinSpeed{0};

  /// Maximum speed value.
  static constexpr int kMaxSpeed{255};

  /// PWM output connected to motor enable pin.
  int enable_pwm_out_{0};

  /// Digital output connected to motor forward pin.
  int forward_out_{0};

  /// Digital output connected to motor backward pin.
  int backward_out_{0};
};
}  // namespace andino
