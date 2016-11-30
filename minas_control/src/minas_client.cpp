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

/*
  See support.robotiq.com -> manual for the register output meanings
*/
void MinasClient::writeOutputs(const MinasOutput& output)
{
  uint8_t map[9] = {0}; // array containing all 15 output registers

  map[0] = (output.controlword) & 0x00ff;
  map[1] = (output.controlword >> 8) & 0x00ff;
  map[2] = output.operation_mode;
  map[3] = (output.target_position) & 0x00ff;
  map[4] = (output.target_position >>  8) & 0x00ff;
  map[5] = (output.target_position >> 16) & 0x00ff;
  map[6] = (output.target_position >> 24) & 0x00ff;
  map[7] = (output.touch_probe_function) & 0x00ff;
  map[8] = (output.touch_probe_function >> 8) & 0x00ff;

  for (unsigned i = 0; i < 9; ++i)
  {
    manager_.write(slave_no_, i, map[i]);
  }
}

MinasInput MinasClient::readInputs() const
{
  MinasInput input;

  uint8_t map[23];
  for (unsigned i = 0; i < 23; ++i)
  {
    map[i] = manager_.readInput(slave_no_, i);
  }

  input.error_code			= *(uint16 *)(map+0);
  input.statusword			= *(uint16 *)(map+2);
  input.operation_mode			= *(uint8  *)(map+4);
  input.position_actual_value		= *(uint32 *)(map+5);
  input.touch_probe_status		= *(uint16 *)(map+9);
  input.touch_probe_posl_pos_value	= *(uint32 *)(map+11);
  input.following_error_acutal_value	= *(uint32 *)(map+15);
  input.digital_inputs			= *(uint32 *)(map+19);

  if (input.error_code >> 8 == 0xff) {
    printf("ERROR : %d\n", (input.error_code)&0x00ff);
  }
  // 6-4 Statusword(6041h)
  if (((input.statusword) & 0x004f) == 0x0000) { // x0xx 0000
    printf("Not ready to switch on\n");
  }else if (((input.statusword) & 0x004f) == 0x0040) { // x1xx 0000
    printf("Switch on disabled\n");
  }else if (((input.statusword) & 0x006f) == 0x0021) { // x01x 0001
    printf("Ready to switch on\n");
  }else if (((input.statusword) & 0x004f) == 0x0023) { // x01x 0011
    printf("Switched on\n");
  }else if (((input.statusword) & 0x004f) == 0x0027) { // x01x 0111
    printf("Operation enabled\n");
  }else if (((input.statusword) & 0x004f) == 0x0007) { // x00x 0111
    printf("Quick stop active\n");
  }else if (((input.statusword) & 0x004f) == 0x000f) { // x0xx 1111
    printf("Fault reaction active\n");
  }else if (((input.statusword) & 0x004f) == 0x0008) { // x0xx 1000
    printf("Fault\n");
  }else{
    printf("Unknown status %04x\n", input.statusword);
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
