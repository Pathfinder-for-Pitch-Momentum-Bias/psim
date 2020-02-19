# Gomspace test case. Gets cycle count purely for diagnostic purposes and logs
# any other Gomspace state fields.
from .base import SingleSatOnlyCase


class GomspaceCheckoutCase(SingleSatOnlyCase):

    def setup_case_singlesat(self, simulation):
        simulation.flight_controller.write_state(
            "pan.state", 9)  # Manual state
        self.run_case_singlesat(simulation)
        print("Gomspace cases finished.")

    def str_to_bool(self, str):
        if str == "true":
            return True
        return False

    def run_case_singlesat(self, simulation):
        simulation.cycle_no = simulation.flight_controller.read_state(
            "pan.cycle_no")

        def read_state(self, string_state):
            return simulation.flight_controller.read_state(string_state)

        def write_state(self, string_state, state_value):
            simulation.flight_controller.write_state(string_state, state_value)
            return read_state(self, string_state)

        # readable fields

        vboost = [int(read_state(self, "gomspace.vboost.output" + str(i)))
                  for i in range(1, 4)]
        for n in range(0, len(vboost)):
            print("vboost" + str(n) + " is: " + str(vboost[n]) + " mV")

        vbatt = int(read_state(self, "gomspace.vbatt"))
        if vbatt < 6000 or vbatt > 8400:
            print(
                "Vbatt is out of expected range [6000, 8400] mV at: " + str(vbatt))

        curin = [int(read_state(self, "gomspace.curin.output" + str(i)))
                 for i in range(1, 4)]
        for n in range(0, len(curin)):
            print("curin" + str(n) + " is: " + str(curin[n]) + " mA")
        cursun = int(read_state(self, "gomspace.cursun"))
        print("cursun is: " + str(cursun) + " mA")
        cursys = int(read_state(self, "gomspace.cursys"))
        print("cursys is: " + str(cursys) + " mA")

        curout = [int(read_state(self, "gomspace.curout.output" + str(i)))
                  for i in range(1, 7)]
        for n in range(0, len(curout)):
            print("curout" + str(n) + " is: " + str(curout[n]) + " mA")

        output = [self.str_to_bool(read_state(self, "gomspace.output.output" + str(i)))
                  for i in range(1, 7)]

        for n in range(0, len(output)):
            out_n = output[n]
            # checked umbilical for which outputs should be 0 and 1:
            # 1-5 are all 5V, 6 is 3.3V
            if out_n is False:
                print("Output-" + str(n) + " is not on")

        wdt_i2c_time_left = int(read_state(self, "gomspace.wdt_i2c_time_left"))
        if wdt_i2c_time_left < 99:
            print("wdt_i2c_time_left is less than 99 seconds at: " +
                  str(wdt_i2c_time_left))

        counter_wdt_i2c = read_state(self, "gomspace.counter_wdt_i2c")
        print("counter_wdt_i2c is: " + str(counter_wdt_i2c))
        counter_boot = read_state(self, "gomspace.counter_boot")
        print("counter_wdt_i2c is: " + str(counter_boot))

        temp = [int(read_state(self, "gomspace.temp.output" + str(i)))
                for i in range(1, 5)]
        for n in range(0, len(temp)):
            temp_n = temp[n]
            if temp_n < 20 or out_n > 25:
                print("Temp-" + str(n) +
                      " is out of room temperature range [20, 25] degC at: " + str(temp_n))

        bootcause = read_state(self, "gomspace.bootcause")
        print("bootcause is: " + str(bootcause))
        battmode = read_state(self, "gomspace.battmode")
        print("battmode is: " + str(battmode))
        pptmode = read_state(self, "gomspace.pptmode")
        print("pptmode is: " + str(pptmode))

        # writable fields
        power_cycle_output_cmd = [self.str_to_bool(read_state(self, "gomspace.power_cycle_output" + str(i) + "_cmd"))
                                  for i in range(1, 7)]
        cycle_no_init = int(read_state(self, "pan.cycle_no"))
        cycle_no = cycle_no_init

        # power cycling
        # start power cycle
        power_cycle_output_cmd = [self.str_to_bool(write_state(self, "gomspace.power_cycle_output"
                                                               + str(i) + "_cmd", "true"))
                                  for i in range(1, 7)]
        # wait for outputs to be off
        while (not all(out == False for out in output)) and cycle_no - cycle_no_init < 600:
            output = [self.str_to_bool(read_state(self, "gomspace.output.output" + str(i)))
                      for i in range(1, 7)]
            simulation.flight_controller.write_state(
                self, "pan.cycle_no", cycle_no + 1)
            cycle_no = int(read_state(self, "pan.cycle_no"))
            if cycle_no - cycle_no_init == 600:
                print(
                    "Power cycled outputs could not turn off after 600 cycles (1 minute)")
        # wait for outputs to turn on again
        while (not all(out == True for out in output)) and cycle_no - cycle_no_init < 600:
            output = [self.str_to_bool(read_state(self, "gomspace.output.output" + str(i)))
                      for i in range(1, 7)]
            simulation.flight_controller.write_state(
                "pan.cycle_no", cycle_no + 1)
            cycle_no = int(read_state(self, "pan.cycle_no"))
            if cycle_no - cycle_no_init == 600:
                print(
                    "Power cycled outputs could not turn on after 600 cycles (1 minute)")
        # check if finished power cycling
        power_cycle_output_cmd = [self.str_to_bool(read_state(self, "gomspace.power_cycle_output"
                                                              + str(i) + "_cmd"))
                                  for i in range(1, 7)]
        for n in range(0, len(power_cycle_output_cmd)):
            if power_cycle_output_cmd[n] == True:
                print("Could not update power_cycle_output" + str(n))

        ppt_mode_cmd = int(read_state(self, "gomspace.pptmode_cmd"))
        ppt_mode_updated = int(write_state(
            self, "gomspace.pptmode_cmd", (int(ppt_mode_cmd) + 1) % 2))
        if ppt_mode_cmd == ppt_mode_updated:
            print("Could not update pptmode")

        heater_cmd = self.str_to_bool(read_state(self, "gomspace.heater_cmd"))
        heater_cmd_updated = self.str_to_bool(write_state(
            self, "gomspace.heater_cmd", not heater_cmd))
        if heater_cmd == heater_cmd_updated:
            print("Could not update heater")

        counter_reset_cmd = self.str_to_bool(read_state(
            self, "gomspace.counter_reset_cmd"))
        counter_reset_cmd_updated = self.str_to_bool(write_state(
            self, "gomspace.counter_reset_cmd", not counter_reset_cmd))
        if counter_reset_cmd == counter_reset_cmd_updated:
            print("Could not update counter_reset")

        gs_reset_cmd = self.str_to_bool(
            read_state(self, "gomspace.gs_reset_cmd"))
        gs_reset_cmd_updated = self.str_to_bool(write_state(
            self, "gomspace.gs_reset_cmd", not gs_reset_cmd))
        if gs_reset_cmd == gs_reset_cmd_updated:
            print("Could not update gs_reset")

        gs_reboot_cmd = self.str_to_bool(
            read_state(self, "gomspace.gs_reboot_cmd"))
        gs_reboot_cmd_updated = self.str_to_bool(write_state(
            self, "gomspace.gs_reboot_cmd", not gs_reboot_cmd))
        if gs_reboot_cmd == gs_reboot_cmd_updated:
            print("Could not update gs_reboot")