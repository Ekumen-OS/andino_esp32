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
#include "pid.h"

namespace andino {
void Pid::reset(int encoder_count) {
  // Since we can assume that the PID is only turned on when going from stop to
  // moving, we can init
  // everything on zero.
  setpoint_ = 0;
  integral_term_ = 0;
  last_enconder_count_ = encoder_count;
  last_input_ = 0;
  last_output_ = 0;
}

void Pid::enable() { enabled_ = true; }

bool Pid::enabled() { return enabled_; }

void Pid::disable() { enabled_ = false; }

void Pid::compute(int encoder_count, int& computed_output) {
  if (!enabled_) {
    if (last_input_ != 0) {
      reset(encoder_count);
    }
    return;
  }

  int input = encoder_count - last_enconder_count_;
  long error = setpoint_ - input;

  long output = (kp_ * error - kd_ * (input - last_input_) + integral_term_) / ko_;
  output += last_output_;

  if (output >= output_max_) {
    output = output_max_;
  } else if (output <= output_min_) {
    output = output_min_;
  } else {
    integral_term_ += ki_ * error;
  }

  computed_output = output;

  last_enconder_count_ = encoder_count;
  last_input_ = input;
  last_output_ = output;
}

void Pid::set_setpoint(int setpoint) { setpoint_ = setpoint; }

void Pid::set_tunings(int kp, int kd, int ki, int ko) {
  kp_ = kp;
  kd_ = kd;
  ki_ = ki;
  ko_ = ko;
}
}  // namespace andino
