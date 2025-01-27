import numpy as np
import matplotlib.pyplot as plt
from control_tools.controllers import PIController

class PVBESSController(PIController):

    def __init__(self, kp, ki, delta_t):
        # Call the parent class's initializer
        super().__init__(kp, ki, delta_t)
        self.command_pv=0
        self.command_bess=0
        
    def calculate_max_pv_power(self, pcc_power_out,max_grid_allowed,nominal_pv_power,max_pv_power_lim=np.inf):
        # Find new max allowed pv power with integral corrections
        new_max_pv_power = nominal_pv_power + self.integral_action(max_grid_allowed,pcc_power_out)
        # Saturate allowed pv power nominal value <=PV_max <= max_value_possible
        new_max_pv_power=max(nominal_pv_power, min(new_max_pv_power, max_pv_power_lim)) #uncomment this line to use saturation
        return  new_max_pv_power
    
    
    def calculate_bess_setpoint(self, pcc_power_out,max_grid_allowed,nominal_bess_power):
        # Find new bess set point with integral corrections
        bess_setpoint =  self.integral_action(max_grid_allowed,pcc_power_out)
        # Saturate power setpoint <=PV_max <= max_value_possible
        bess_setpoint = max(-nominal_bess_power, min(bess_setpoint, nominal_bess_power))
        return  bess_setpoint
    
    def calculate_controller_output(self,pcc_power_out,pv_power_out,bess_power_out, bess_soc,max_grid_allowed,nominal_pv_power,nominal_bess_power):
        grid_allowed_error=max_grid_allowed-pcc_power_out
        soc_min=5; # Minimum SOC for BESS
        soc_max=95; # Maximum SOC for BESS
        # Calculate max PV power
        if grid_allowed_error >0 or bess_soc>=soc_min or np.abs(bess_power_out)>=nominal_bess_power: # Prioritize grid injection if possible
            new_max_pv_power=self.calculate_max_pv_power(pcc_power_out,max_grid_allowed,nominal_pv_power) 
        else: # BESS to be used
            new_max_pv_power=self.command_pv
    
        # Calcularte BESS power setpoint
        bess_setpoint =  self.integral_action(max_grid_allowed,pcc_power_out)
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
    

class BESS:
    def __init__(self,  nominal_power, initial_soc, delta_t=1, tau=2):
        self.soc = initial_soc
        self.bess_power = 0
        self.alpha = delta_t / tau
        self.delta_t = delta_t
        self.nominal_power = nominal_power
