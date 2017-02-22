/****************************************************************************
# minas_client.cpp:  MINAS A5B EtherCAT Motor Controller                    #
# Copyright (C) 2017, Tokyo Opensource Robotics Kyokai Association          #
#                                                                           #
# This program is free software; you can redistribute it and/or modify      #
# it under the terms of the GNU General Public License as published by      #
# the Free Software Foundation; either version 2 of the License, or         #
# (at your option) any later version.                                       #
#                                                                           #
# This program is distributed in the hope that it will be useful,           #
# but WITHOUT ANY WARRANTY; without even the implied warranty of            #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
# GNU General Public License for more details.                              #
#                                                                           #
# You should have received a copy of the GNU General Public License         #
# along with this program; if not, write to the Free Software               #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA #
#                                                                           #
****************************************************************************/

#include <minas_control/minas_hardware_interface.h>
#include <getopt.h>
#include <boost/foreach.hpp>

namespace minas_control
{

#define PULSE_PER_REVOLUTE (1048576 / (2 * M_PI) ) // 20 bit
  //#define PULSE_PER_REVOLUTE ( 131072 / (2 * M_PI) )// 17 bit

  EtherCATJointControlInterface::EtherCATJointControlInterface(ethercat::EtherCatManager* manager, int slave_no, hardware_interface::JointStateInterface& jnt_stat, hardware_interface::PositionJointInterface& jnt_cmd) : JointControlInterface(slave_no, jnt_stat, jnt_cmd)
  {

    // EtherCAT
    int operation_mode = 0x08; // (csp) cyclic synchronous position mode

    client = new MinasClient(*manager, slave_no);

    client->reset();

    // set paramete from PANATERM test program
    client->setTrqueForEmergencyStop(100); // 100%
    client->setOverLoadLevel(50);          // 50%
    client->setOverSpeedLevel(120);        // r/min
    client->setMotorWorkingRange(0.1);     // 0.1

    client->setInterpolationTimePeriod(4000);     // 4 msec

    // servo on
    client->servoOn();

    // get current positoin
    input = client->readInputs();
    int32 current_position = input.position_actual_value;

    // set target position
    memset(&output, 0x00, sizeof(minas_control::MinasOutput));
    if ( operation_mode == 0x01 )
      { // (pp) position profile mode
	output.target_position = (current_position > 0)?(current_position - 0x100000):(current_position + 0x100000);
      }
    else
      { // (csp) cyclic synchronous position mode
	//output.target_position = current_position;
	output.target_position = 0;
	output.position_offset = current_position;
      }
    output.max_motor_speed = 120;  // rad/min
    output.target_torque = 500;    // 0% (unit 0.1%)
    output.max_torque    = 500;    // 50% (unit 0.1%)
    output.controlword   = 0x001f; // move to operation enabled + new-set-point (bit4) + change set immediately (bit5)

    // change to cyclic synchronous position mode
    output.operation_mode = operation_mode;
    //output.operation_mode = 0x08; // (csp) cyclic synchronous position mode
    //output.operation_mode = 0x01; // (pp) position profile mode

    // set profile velocity
    client->setProfileVelocity(0x20000000);

    // pp control model setup (see statusword(6041.h) 3) p.107)
    client->writeOutputs(output);
    while ( ! (input.statusword & 0x1000) ) {// bit12 (set-point-acknowledge)
      input = client->readInputs();
    }
    output.controlword   &= ~0x0010; // clear new-set-point (bit4)
    client->writeOutputs(output);

    ROS_WARN("target position = %08x", output.target_position);
    ROS_WARN("position offset = %08x", output.position_offset);
    joint.cmd_ = joint.pos_ = current_position / (PULSE_PER_REVOLUTE);
    joint.vel_ = joint.eff_ = 0;
  }

  EtherCATJointControlInterface::~EtherCATJointControlInterface()
  {
    ROS_INFO_STREAM_NAMED("minas", "~EtherCATJointControlInterface()");
    shutdown();
    delete(client);
  }

  void EtherCATJointControlInterface::shutdown()
  {
    ROS_INFO_STREAM_NAMED("minas", joint.name_ + " shutdown()");
    client->printPDSStatus(input);
    client->printPDSOperation(input);
    client->servoOff();
  }

  void EtherCATJointControlInterface::read()
  {
    input = client->readInputs();
    output = client->readOutputs();
    joint.pos_ = int32_t(input.position_actual_value) / PULSE_PER_REVOLUTE;
    joint.vel_ = int32_t(input.velocity_actual_value) / PULSE_PER_REVOLUTE;
    joint.eff_ = int32_t(input.torque_actual_value) / PULSE_PER_REVOLUTE;
  }

  void EtherCATJointControlInterface::write()
  {
    output.position_offset = uint32_t(joint.cmd_ * PULSE_PER_REVOLUTE);
    client->writeOutputs(output);
  }

  //
  DummyJointControlInterface::DummyJointControlInterface(int slave_no, hardware_interface::JointStateInterface& jnt_stat, hardware_interface::PositionJointInterface& jnt_cmd) : JointControlInterface(slave_no, jnt_stat, jnt_cmd) {
    joint.cmd_ = joint.pos_ = joint.vel_ = joint.eff_ = 0;
  }

  void DummyJointControlInterface::read()
  {
    joint.pos_ = joint.cmd_;
    joint.vel_ = 0;
    joint.eff_ = 0;
  }

  void DummyJointControlInterface::write()
  {
  }

  //
  MinasHardwareInterface::MinasHardwareInterface(std::string ifname, bool in_simulation)
    : manager(NULL)
  {
    /* start MinasClient */
    if (in_simulation) {
      ROS_INFO_STREAM_NAMED("minas","Minas Hardware Interface in simulation mode");
      for (int i = 1; i <= 6; i++ ) {
	registerControl(new DummyJointControlInterface(i,
						       joint_state_interface,
						       joint_position_interface
						       ));
      }
    } else {
    
      manager = new ethercat::EtherCatManager(ifname);

      n_dof_ = manager->getNumClinets();
      if ( n_dof_ != 6 ) {
	ROS_ERROR_STREAM_NAMED("minas", "Minas Hardware Interface expecting 6 clients");
      }
      int i;
      for (i = 1; i <= n_dof_; i++ )
	{
	  registerControl(new EtherCATJointControlInterface(manager, i,
							    joint_state_interface,
							    joint_position_interface
							    ));
	}
      for (; i <= 6; i++ )
	{
	  registerControl(new DummyJointControlInterface(i,
							 joint_state_interface,
							 joint_position_interface
							 ));
	  ROS_ERROR_STREAM_NAMED("minas", "Could not find EtherCAT client");
	  ROS_ERROR_STREAM_NAMED("minas", "Minas Hardware Interface uses Dummy joint " << i);
	}
    }

    registerInterface(&joint_state_interface);
    registerInterface(&joint_position_interface);
  }

  MinasHardwareInterface::~MinasHardwareInterface()
  {
    shutdown();
  }

  void MinasHardwareInterface::shutdown()
  {
    BOOST_FOREACH (JointControlInterface* control, controls) {
      control->shutdown();
    }
    controls.clear();
    if ( manager != NULL ) {
      ROS_INFO_STREAM_NAMED("minas", "Delete manager");
      delete(manager);
    }
    manager = NULL;
  }

  void MinasHardwareInterface::registerControl(JointControlInterface* control)
  {
    controls.push_back(control);
  }

  bool MinasHardwareInterface::read(const ros::Time time, const ros::Duration period)
  {
    BOOST_FOREACH (JointControlInterface* control, controls) {
      control->read();
    }
  }

  void MinasHardwareInterface::write(const ros::Time time, const ros::Duration period)
  {
    BOOST_FOREACH (JointControlInterface* control, controls) {
      control->write();
    }
  }

  inline ros::Time MinasHardwareInterface::getTime()
  {
    return ros::Time::now();
  }

  inline ros::Duration MinasHardwareInterface::getPeriod()
  {
    return ros::Duration(0.001);
  }

} // namespace
