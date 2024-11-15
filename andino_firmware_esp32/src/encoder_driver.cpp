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
#include "encoder_driver.h"

namespace andino {

EncoderDriver::EncoderDriver(const int& encoder_channel_a, const int& encoder_channel_b)
    : encoder_channel_a_{encoder_channel_a}, encoder_channel_b_{encoder_channel_b} {}

void EncoderDriver::begin() {
  pinMode(encoder_channel_a_, INPUT);
  pinMode(encoder_channel_b_, INPUT);
}

void IRAM_ATTR EncoderDriver::callback() {
  previous_state_ = current_state_;

  if (digitalRead(encoder_channel_a_))
    bitSet(current_state_, 1);
  else
    bitClear(current_state_, 1);

  if (digitalRead(encoder_channel_b_))
    bitSet(current_state_, 0);
  else
    bitClear(current_state_, 0);

  if (previous_state_ == 2 && current_state_ == 0) instance_count_--;
  if (previous_state_ == 0 && current_state_ == 1) instance_count_--;
  if (previous_state_ == 3 && current_state_ == 2) instance_count_--;
  if (previous_state_ == 1 && current_state_ == 3) instance_count_--;

  if (previous_state_ == 1 && current_state_ == 0) instance_count_++;
  if (previous_state_ == 3 && current_state_ == 1) instance_count_++;
  if (previous_state_ == 0 && current_state_ == 2) instance_count_++;
  if (previous_state_ == 2 && current_state_ == 3) instance_count_++;
}

int EncoderDriver::read() {
  int instance_count = 0;
  noInterrupts();
  instance_count = instance_count_;
  interrupts();
  return instance_count;
}
}  // namespace andino
