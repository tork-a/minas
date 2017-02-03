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
  uint16 error_code;			// 603Fh : Error code
  uint16 statusword;			// 6041h : Statusword
  uint8  operation_mode;		// 6061h : Modes of operation display
  uint32 position_actual_value;		// 6064h : Position actual value
  uint32 velocity_actual_value;		// 606Ch : Velocity actual value
  uint16 torque_actual_value;		// 6077h : Torque actual value
  uint16 touch_probe_status;		// 60B9h : Touch probe status
  uint32 touch_probe_posl_pos_value;	// 60BAh : Touch probe pos1 pos value
  uint32 digital_inputs;		// 60FDh : Digital inputs
} MinasInput;

typedef struct {
  uint16 controlword;			// 6040h : Controlword
  uint8  operation_mode;		// 6060h : Mode of operation
  uint16 target_torque;			// 6071h : Target Torque
  uint16 max_torque;			// 6072h : Max Torque
  uint32 target_position;		// 607Ah : Target Position
  uint32 max_motor_speed;		// 6080h : Max motor speed
  uint16 touch_probe_function;		// 60B8h : Touch Probe function
  uint32 target_velocity;		// 60FFh : Target Velocity
  uint32 position_offset;		// 60B0h : Position Offset
} MinasOutput;

typedef enum {NOT_READY, SWITCH_DISABLED, READY_SWITCH, SWITCHED_ON, OPERATION_ENABLED, QUICK_STOP, FAULT_REACTION, FAULT, UNKNOWN} PDS_STATUS;  // Statusword(6041h) SX-DSV02470 p.78

typedef enum {NO_MODE_CHANGE, PROFILE_POSITION_MODE, VELOCITY_MODE, PROFILE_VELOCITY_MODE, TORQUE_PROFILE_MODE, HOMING_MODE, INTERPOLATED_POSITION_MODE, CYCLIC_SYNCHRONOUS_POSITION_MODE, CYCLIC_SYNCHRONOUS_VELOCITY_MODE, CYCLIC_SYNCHRONOUS_TORQUE_MODE} PDS_OPERATION; // Mode of operation(6061h) SX-DSV02470 p.83

 typedef enum {//HALT, FAULT_RESET, ENABLE_OPERATION, QUICK_STOP, ENABLE_VOLTAGE, SWITCH_ON, 
} PDS_CONTROL; // Controlworld(6040h) SX-DSV02470 p.76

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

  /*
   * \brief Torque setup for emergency stop [%] 0 - 500 (3511h / 00h)
   * \return void
   */
  void setTrqueForEmergencyStop(double val);

  /*
   * \brief Over-load level setup [%] 0 - 500 (3512h / 00h)
   * \return void
   */
  void setOverLoadLevel(double val);

  /*
   * \brief Over-speed level setup [r/min] 0 - 2000 (3513h / 00h)
   * \return void
   */
  void setOverSpeedLevel(double val);

  /*
   * \brief Motor working range setup [revolution] 0 - 100 (3514h / 00h)
   * \return void
   */
  void setMotorWorkingRange(double val);

  /*
   * \brief set Profile velocity 0 - 4294967295 (6081h / 00h)
   * \return void
   */
  void setProfileVelocity(uint32_t val);

  /*
   * \brief set Interpolation Time Period 250, 500, 1000, 2000, 4000 us
   * \return void
   */
  void setInterpolationTimePeriod(int us);

  /**
   * \brief print status from input data
   */
  void printPDSStatus(const MinasInput input) const;

  /**
   * \brief print operation mode from input data
   */
  void printPDSOperation(const MinasInput input) const;

  /**
   * \brief print control status from input data
   */
  void printPDSControl(const MinasInput input) const;

private:
  /**
   * \brief get status from input data
   * \return status
   */
  PDS_STATUS getPDSStatus(const MinasInput input) const;

  /**
   * \brief get operation mode from input data
   * \return status
   */
  PDS_OPERATION getPDSOperation(const MinasInput input) const;

  /**
   * \brief get control status from input data
   * \return status
   */
  PDS_STATUS getPDSControl(const MinasInput input) const;

  ethercat::EtherCatManager& manager_;
  const int slave_no_;
};

}

#endif
