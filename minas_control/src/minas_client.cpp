/*
 * Copyright (C) 2016, Tokyo Opensource Robotics Kyokai Association
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Tokyo Opensource Robotics Kyokai Association. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <minas_control/minas_client.h>

// An effort to keep the lines less than 100 char long
namespace minas_control
{
MinasClient::MinasClient(ethercat::EtherCatManager& manager, int slave_no)
  : manager_(manager)
  , slave_no_(slave_no)
{}

void MinasClient::writeOutputs(const MinasOutput& output)
{
  uint8_t map[21] = {0}; // array containing all 15 output registers

  map[0] = (output.controlword) & 0x00ff;
  map[1] = (output.controlword >> 8) & 0x00ff;
  map[2] = output.operation_mode;
  map[3] = (output.target_torque) & 0x00ff;
  map[4] = (output.target_torque >>  8) & 0x00ff;
  map[5] = (output.max_torque) & 0x00ff;
  map[6] = (output.max_torque >>  8) & 0x00ff;
  map[7] = (output.target_position) & 0x00ff;
  map[8] = (output.target_position >>  8) & 0x00ff;
  map[9] = (output.target_position >> 16) & 0x00ff;
  map[10] = (output.target_position >> 24) & 0x00ff;
  map[11] = (output.max_motor_speed) & 0x00ff;
  map[12] = (output.max_motor_speed >>  8) & 0x00ff;
  map[13] = (output.max_motor_speed >> 16) & 0x00ff;
  map[14] = (output.max_motor_speed >> 24) & 0x00ff;
  map[15] = (output.touch_probe_function) & 0x00ff;
  map[16] = (output.touch_probe_function >> 8) & 0x00ff;
  map[17] = (output.target_velocity) & 0x00ff;
  map[18] = (output.target_velocity >>  8) & 0x00ff;
  map[19] = (output.target_velocity >> 16) & 0x00ff;
  map[20] = (output.target_velocity >> 24) & 0x00ff;

  for (unsigned i = 0; i < 21; ++i)
  {
    manager_.write(slave_no_, i, map[i]);
  }
}

MinasInput MinasClient::readInputs() const
{
  MinasInput input;
  uint8_t map[25];
  for (unsigned i = 0; i < 25; ++i)
  {
    map[i] = manager_.readInput(slave_no_, i);
  }

  input.error_code			= *(uint16 *)(map+0);
  input.statusword			= *(uint16 *)(map+2);
  input.operation_mode			= *(uint8  *)(map+4);
  input.position_actual_value		= *(uint32 *)(map+5);
  input.velocity_actual_value		= *(uint32 *)(map+9);
  input.torque_actual_value		= *(uint16 *)(map+13);
  input.touch_probe_status		= *(uint16 *)(map+15);
  input.touch_probe_posl_pos_value	= *(uint32 *)(map+17);
  input.digital_inputs			= *(uint32 *)(map+21);

  if (input.error_code >> 8 == 0xff) {
    printf("ERROR : %d\n", (input.error_code)&0x00ff);
  }

  return input;
}

MinasOutput MinasClient::readOutputs() const
{

  MinasOutput output;

  uint8_t map[9];
  for (unsigned i = 0; i < 9; ++i)
  {
    map[i] = manager_.readOutput(slave_no_, i);
  }

  output.controlword			= *(uint16 *)(map+0);
  output.operation_mode			= *(uint8  *)(map+2);
  output.target_position		= *(uint32 *)(map+3);
  output.touch_probe_function		= *(uint16 *)(map+7);

  return output;
}
} // end of minas_control namespace
