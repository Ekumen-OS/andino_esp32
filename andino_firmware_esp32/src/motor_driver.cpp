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
#include "motor_driver.h"

namespace andino {
MotorDriver::MotorDriver(const int& enable_pwm_out, const int& forward_out, const int& backward_out)
    : enable_pwm_out_{enable_pwm_out}, forward_out_{forward_out}, backward_out_{backward_out} {}

void MotorDriver::begin() {
  pinMode(enable_pwm_out_, OUTPUT);
  pinMode(forward_out_, OUTPUT);
  pinMode(backward_out_, OUTPUT);
}

void MotorDriver::set_speed(int speed) {
  bool forward = true;

  if (speed > kMaxSpeed) {
    speed = kMaxSpeed;
  }

  if (speed < kMinSpeed) {
    speed = -speed;
    forward = false;
  }

  if (forward) {
    digitalWrite(backward_out_, HIGH);
    digitalWrite(forward_out_, LOW);
  } else {
    digitalWrite(backward_out_, LOW);
    digitalWrite(forward_out_, HIGH);
  }

  analogWrite(enable_pwm_out_, speed);
}
}  // namespace andino
