import numpy as np
import matplotlib.pyplot as plt
from control_tools.controllers import PIController

class PVBESSController(PIController):

    def __init__(self, kp, ki, delta_t):
        # Call the parent class's initializer
        super().__init__(kp, ki, delta_t)
        self.command_pv=0
        self.command_bess=0

    
    def calculate_controller_output(self,pcc_power_out,pv_power_out,bess_power_out, bess_soc,max_grid_allowed,nominal_pv_power,nominal_bess_power,soc_min=5 ,soc_max=95,max_pv_power_lim=np.inf):

        load_power_observations=pcc_power_out-pv_power_out-bess_power_out
        # Calculate max PV power

        if pcc_power_out<max_grid_allowed: # Prioritize grid injection if power excess is below grid limit
            new_max_pv_power=nominal_pv_power + self.integral_action(max_grid_allowed,pcc_power_out)
            if pv_power_out +load_power_observations<0:
                bess_setpoint = -(pv_power_out+load_power_observations) # Discharge BESS to cover load
            else:
                bess_setpoint = self.command_bess
            
        else: # BESS to be charged if power excess grid limits
            # Calculate BESS power setpoint
            bess_setpoint = self.integral_action(max_grid_allowed,pcc_power_out)
            if bess_soc>=soc_max or np.abs(bess_setpoint)>=nominal_bess_power: # Check if possible to charge
                new_max_pv_power = nominal_pv_power + self.integral_action(max_grid_allowed,pcc_power_out)
                bess_setpoint = max(-nominal_bess_power, min(bess_setpoint, nominal_bess_power)) 
            else:
                new_max_pv_power = self.command_pv

        if bess_setpoint >0 and bess_soc<=soc_min: # Check if possible to discharge
            bess_setpoint=0
            self.reset_integral()
        elif bess_setpoint <0 and bess_soc>=soc_max: # Check if possible to charge
            bess_setpoint=0
            self.reset_integral()

        # Update command values
        self.command_pv=new_max_pv_power
        self.command_bess=bess_setpoint
        return new_max_pv_power,bess_setpoint
    


