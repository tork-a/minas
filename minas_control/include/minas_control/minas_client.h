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

#ifndef MINAS_CLIENT_H
#define MINAS_CLIENT_H

#include <soem/osal.h>
#include <ethercat_manager/ethercat_manager.h>

// Forward declaration of EtherCatManager
namespace ethercat
{
class EtherCatManager;
}

namespace minas_control
{

/**
 * \brief This class provides a client for the EtherCAT manager object that
 *        can translate robot input/output messages and translate them to
 *        the underlying IO Map.
 */

// Default PDO maping 4
// Position contorl, Velocity COntrol, Torque Controle (Touch proble, torque limit)
typedef struct {
  // input
  uint16 error_code;
  uint16 statusword;
  uint8  operation_mode;
  uint32 position_actual_value;
  uint32 velocity_actual_value;
  uint16 torque_actual_value;
  uint16 touch_probe_status;
  uint32 touch_probe_posl_pos_value;
  uint32 digital_inputs;
} MinasInput;

typedef struct {
  uint16 controlword;
  uint8  operation_mode;
  uint16 target_torque;
  uint16 max_torque;
  uint32 target_position;
  uint32 max_motor_speed;
  uint16 touch_probe_function;
  uint32 target_velocity;
} MinasOutput;

typedef enum {NOT_READY, SWITCH_DISABLED, READY_SWITCH, SWITCHED_ON, OPERATION_ENABLED, QUICK_STOP, FAULT_REACTION, FAULT, UNKNOWN} PDS_STATUS;

class MinasClient
{
public:
  /**
   * \brief Constructs a control interface to a MINUS AC Servo on
   *        the given ethercat network and the given slave_no.
   *
   * @param[in] manager The interface to an EtherCAT network that the gripper
   *                    is connected to.
   *
   * @param[in] slave_no The slave number of the gripper on the EtherCAT network
   *                     (>= 1)
   */
  MinasClient(ethercat::EtherCatManager& manager, int slave_no);

  /**
   * \brief Write the given set of control flags to the memory of the controller
   *
   * @param[in] output The set of output-register values to write to the gripper
   */
  void writeOutputs(const MinasOutput& output);

  /**
   * \brief Reads set of input-register values from the controller.
   * \return The AC servo input registers as read from the controller IOMap
   */
  MinasInput readInputs() const;

  /**
   * \brief Reads set of output-register values from the controller.
   * \return The AC servo output registers as read from the controller IOMap
   */
  MinasOutput readOutputs() const;

  /**
   * \brief Reset alarm
   * \return void
   */
  void reset();

  /**
   * \brief Send servo on sequence to the controller
   * \return void
   */
  void servoOn();

  /**
   * \brief Send servo off sequence to the controller
   * \return void
   */
  void servoOff();

private:
  /**
   * \brief get status from input data
   * \return status
   */
  PDS_STATUS getPDSStatus(const MinasInput input) const;

  /**
   * \brief print status from input data
   */
  void printPDSStatus(const MinasInput input) const;

  ethercat::EtherCatManager& manager_;
  const int slave_no_;
};

}

#endif
