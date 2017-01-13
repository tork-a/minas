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

void MinasClient::reset()
{
  MinasInput input = readInputs();
  if ( input.error_code == 0 ) return;

  // section 8.4 of SX-DSV02470 --248--
  MinasOutput output;
  memset(&output, 0x00, sizeof(MinasOutput));
  output.controlword = 0x0080; // fault reset
  writeOutputs(output);

  while ( input.error_code != 0 ) {
    sleep(1);
    printf("Waiting for Fault Reset...\n");
    input = readInputs();
    printf("error_code = %04x, status_word %04x, operation_mode = %2d, position = %08x\n",
	   input.error_code, input.statusword, input.operation_mode, input.position_actual_value);
  }
  printf("Fault was cleared\n");
}

void MinasClient::servoOn()
{
  MinasInput input = readInputs();
  printPDSStatus(input);
  MinasOutput output;
  memset(&output, 0x00, sizeof(MinasOutput));
  output.operation_mode = 1; // pp (profile position mode)
  while (getPDSStatus(input) != OPERATION_ENABLED) {
    switch ( getPDSStatus(input) ) {
      case SWITCH_DISABLED:
	output.controlword = 0x0006; // move to ready to switch on
	break;
      case READY_SWITCH:
	output.controlword = 0x0007; // move to switched on
	break;
      case SWITCHED_ON:
	output.controlword = 0x000f; // move to operation enabled
	break;
      case OPERATION_ENABLED:
	break;
      default:
	printf("unknown status");
	return;
      }
    writeOutputs(output);
    usleep(10*1000);
    input = readInputs();
    printPDSStatus(input);
  }
}

void MinasClient::servoOff()
{
  MinasInput input = readInputs();
  printPDSStatus(input);
  MinasOutput output;
  memset(&output, 0x00, sizeof(MinasOutput));
  while (getPDSStatus(input) != SWITCH_DISABLED) {
    switch ( getPDSStatus(input) ) {
      case READY_SWITCH:
	output.controlword = 0x0000; // disable voltage
	break;
      case SWITCHED_ON:
	output.controlword = 0x0006; // shutdown
	break;
      case OPERATION_ENABLED:
	output.controlword = 0x0007; // disable operation
	break;
      default:
	printf("unknown status");
	output.controlword = 0x0000; // disable operation
	break;
    }
    writeOutputs(output);
    usleep(10*1000);
    input = readInputs();
    printPDSStatus(input);
  }
}

PDS_STATUS MinasClient::getPDSStatus(const MinasInput input) const
{
  uint16 statusword = input.statusword;
  if (((statusword) & 0x004f) == 0x0000) { // x0xx 0000
    return NOT_READY;
  }else if (((statusword) & 0x004f) == 0x0040) { // x1xx 0000
    return SWITCH_DISABLED;
  }else if (((statusword) & 0x006f) == 0x0021) { // x01x 0001
    return READY_SWITCH;
  }else if (((statusword) & 0x006f) == 0x0023) { // x01x 0011
    return SWITCHED_ON;
  }else if (((statusword) & 0x006f) == 0x0027) { // x01x 0111
    return OPERATION_ENABLED;
  }else if (((statusword) & 0x006f) == 0x0007) { // x00x 0111
    return QUICK_STOP;
  }else if (((statusword) & 0x004f) == 0x000f) { // x0xx 1111
    return FAULT_REACTION;
  }else if (((statusword) & 0x004f) == 0x0008) { // x0xx 1000
    return FAULT;
  }else{
    return UNKNOWN;
  }
}

void MinasClient::printPDSStatus(const MinasInput input) const
{
  switch ( getPDSStatus(input) ) {
    case NOT_READY:
      printf("Not ready to switch on\n");
      break;
    case SWITCH_DISABLED:
      printf("Switch on disabled\n");
      break;
    case READY_SWITCH:
      printf("Ready to switch on\n");
      break;
    case SWITCHED_ON:
      printf("Switched on\n");
      break;
    case OPERATION_ENABLED:
      printf("Operation enabled\n");
      break;
    case QUICK_STOP:
      printf("Quick stop active\n");
      break;
    case FAULT_REACTION:
      printf("Fault reaction active\n");
      break;
    case FAULT:
      printf("Fault\n");
      break;
    case UNKNOWN:
      printf("Unknown status %04x\n", input.statusword);
      break;
    }
}

void MinasClient::setTrqueForEmergencyStop(double val)
{
  // 3511h, unit: %, range: 0 - 500, I16
  int16_t i16val = (int16_t)val;
  manager_.writeSDO<int16_t>(slave_no_, 0x3511, 0x00, i16val);
}

void MinasClient::setOverLoadLevel(double val)
{
  // 3512h, unit: %, range: 0 - 500, I16
  int16_t i16val = (int16_t)val;
  manager_.writeSDO<int16_t>(slave_no_, 0x3512, 0x00, i16val);
}

void MinasClient::setOverSpeedLevel(double val)
{
  // 3513h, unit: r/min, range: 0 - 20000, I16
  int16_t i16val = (int16_t)val;
  manager_.writeSDO<int16_t>(slave_no_, 0x3513, 0x00, i16val);
}

void MinasClient::setMotorWorkingRange(double val)
{
  // 3514h, unit: 0.1 revolute, range: 0 - 1000, I16
  int16_t i16val = (int16_t)(val*10);
  manager_.writeSDO<int16_t>(slave_no_, 0x3514, 0x00, i16val);
}

void MinasClient::setProfileVelocity(uint32_t val)
{
  // 6091h, unit: pulse, range: 0 - 4294967295, U32
  uint32_t u32val = (uint32_t)val;
  manager_.writeSDO<uint32_t>(slave_no_, 0x6081, 0x00, u32val);
}

} // end of minas_control namespace
