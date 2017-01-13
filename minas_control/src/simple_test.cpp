/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 */

#include <stdio.h>
#include <ethercat_manager/ethercat_manager.h>
#include <minas_control/minas_client.h>

int main(int argc, char *argv[])
{
  printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

  if (argc > 1)
    {      
      /* start slaveinfo */
      std::string ifname(argv[1]);
      ethercat::EtherCatManager manager(ifname);
      minas_control::MinasClient client(manager, 1);

      // clear error
      client.reset();

      // set paramete from PANATERM test program
      client.setTrqueForEmergencyStop(100); // 100%
      client.setOverLoadLevel(50);          // 50%
      client.setOverSpeedLevel(120);        // r/min
      client.setMotorWorkingRange(0.1);     // 0.1

      // servo on
      client.servoOn();

      // get current positoin
      minas_control::MinasInput input = client.readInputs();
      int32 current_position = input.position_actual_value;

      // set target position
      minas_control::MinasOutput output;
      memset(&output, 0x00, sizeof(minas_control::MinasOutput));
      output.target_position = (current_position > 0)?(current_position - 0x100000):(current_position + 0x100000);
      output.max_motor_speed = 120;  // rad/min
      output.target_torque = 500;    // 0% (unit 0.1%)
      output.max_torque    = 500;    // 50% (unit 0.1%)
      output.controlword   = 0x001f; // move to operation enabled + new-set-point (bit4) + change set immediately (bit5)

      // set profile velocity
      client.setProfileVelocity(0x20000000);

      // pp control model setup (see statusword(6041.h) 3) p.107)
      client.writeOutputs(output);
      while ( ! (input.statusword & 0x1000) ) {// bit12 (set-point-acknowledge)
	input = client.readInputs();
      }
      output.controlword   |= 0x0004; // clear new-set-point (bit4)
      client.writeOutputs(output);

      printf("target position = %08x\n", output.target_position);
      for (int i = 0; i <= 2000; i++ ) {
	input = client.readInputs();
	if ( i % 10 == 0 )
	  {
	    printf("err = %04x, ctrl %04x, status %04x, op_mode = %2d, pos = %08x, vel = %08x, tor = %08x\n",
		   input.error_code, output.controlword, input.statusword, input.operation_mode, input.position_actual_value, input.velocity_actual_value, input.torque_actual_value);
	    if ( input.statusword & 0x0400 ) { // target reached (bit 10)
	      printf("target reached\n");
	      break;
	    }
	  }
        usleep(5000);
      }
      client.servoOff();
    }
  else
    {
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
    }   
   
  printf("End program\n");

  return 0;
}

