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
      minas_control::MinasInput input = client.readInputs();
      if ( input.error_code != 0 ) 
	{
          minas_control::MinasOutput output;
          memset(&output, 0x00, sizeof(minas_control::MinasOutput));
          output.controlword = 0x0080;
          client.writeOutputs(output);
        }
      // loop
      for (int i = 0; i <= 100; i++ ) {
        input = client.readInputs();
        printf("error_code = %04x, status_word %04x, operation_mode = %2d, position = %08x\n",
		 input.error_code, input.statusword, input.operation_mode, input.position_actual_value);

        usleep(5000);
      }

    }
  else
    {
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
    }   
   
  printf("End program\n");

  return 0;
}

