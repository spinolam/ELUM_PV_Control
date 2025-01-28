import numpy as np
import matplotlib.pyplot as plt
from control_tools.controllers import PIController
from control_tools.processes import Inverter, BESS

class PVMaxBESSController():

    def __init__(self, kp, ki, delta_t=1):
        # Call the parent class's initializer
        self.command_pv=0
        self.command_bess=0
        self.controller_bess=PIController(0.1, 0.1, delta_t)
        self.controller_pv=PIController(kp, ki, delta_t)

        

    
    def calculate_controller_output(self,pcc_power_out,pv_power_out,bess_power_out, bess_soc,max_grid_allowed,nominal_pv_power,nominal_bess_power,soc_min=5 ,soc_max=95,max_pv_power_lim=np.inf):
        load_power_observations=pcc_power_out-pv_power_out-bess_power_out
        # Calculate max PV power

        if pcc_power_out<=max_grid_allowed: # Prioritize grid injection if power excess is below grid limit
            new_max_pv_power=nominal_pv_power + self.controller_pv.integral_action(max_grid_allowed,pcc_power_out)
            if pv_power_out +load_power_observations<0:
                bess_setpoint = -(pv_power_out+load_power_observations) # Discharge BESS to cover load
            else:
                bess_setpoint = self.command_bess
                
            
        else: # BESS to be charged if power excess grid limits
            # Calculate BESS power setpoint
            bess_setpoint = self.controller_bess.integral_action(max_grid_allowed,pcc_power_out)
            
            if bess_soc>=soc_max or np.abs(bess_setpoint)>=nominal_bess_power: # Check if possible to charge
                new_max_pv_power = nominal_pv_power + self.controller_pv.integral_action(max_grid_allowed,pcc_power_out)
                bess_setpoint = max(-nominal_bess_power, min(bess_setpoint, nominal_bess_power))
                
            else:
                new_max_pv_power = self.command_pv
               

        if bess_setpoint >0 and bess_soc<=soc_min: # Check if possible to discharge
            bess_setpoint=0
            self.controller_bess.reset_integral()
        elif bess_setpoint <0 and bess_soc>=soc_max: # Check if possible to charge
            bess_setpoint=0
            self.controller_bess.reset_integral()

        # Update command values
        self.command_pv=new_max_pv_power
        self.command_bess=bess_setpoint
        return new_max_pv_power,bess_setpoint
    




# Initialize PI controller
controller = PVMaxBESSController(0.3, 0.3)
inverter_pv = Inverter(1000)
BESS1 = BESS(140)

# Simulation parameters
disturbance = [0] * 30 + [-100] * 30 + [100] * 40  # Constant disturbance (W)
nominal_pv_power=1000
max_grid_injection=1100

# Initialize variables
pv_output = [1000]  # Initial PV power output (W)
pcc_output = [1000]  # PCC power output
bess_output=[100] # Initial BESS power output (W)
bess_soc_output=[50]
pv_command_max=[]
load_observations=[]


for k in range(100):

    load_observations.append(pcc_output[-1]-pv_output[-1]-bess_output[-1])
    pv_command,bess_set_point=controller.calculate_controller_output(pcc_output[-1], pv_output[-1],bess_output[-1],BESS1.soc,max_grid_injection,nominal_pv_power,BESS1.nominal_power)
    pv_output.append(inverter_pv.simulate_sld_discrete(pv_command))
    pv_command_max.append(pv_command)
    bess_output.append(BESS1.simulate_bess_discrete(bess_set_point))
    bess_soc_output.append(BESS1.soc)
    pcc_output.append(disturbance[k]+pv_output[-1]+bess_output[-1])

# Plot results
time = np.arange(0, 100 * 1, 1)
plt.figure(figsize=(10, 6))
plt.plot(time, pv_output[:-1], label="PV Output (W)")
plt.plot(time, pcc_output[:-1], label="PCC Output (W)")
plt.plot(time, bess_output[:-1], label="BESS Output (W)")
plt.plot(time,pv_command_max[:], label="PV commands (W)")
plt.plot(time,load_observations[:], label="Loads (W)")
plt.axhline(max_grid_injection, color="red", linestyle="--", label="Max Grid Injection (W)")
plt.xlabel("Time (s)")
plt.ylabel("Power (W)")
plt.title("Discrete Dynamic Model of SLD (Case Study 2)")
plt.legend()
plt.grid()
plt.show()
plt.figure(figsize=(10, 6))
plt.plot(time, bess_soc_output[:-1], label="BESS SoC Output (W)")
plt.xlabel("Time (s)")
plt.ylabel("State-of-charge (%)")
plt.grid()
plt.title("Discrete Dynamic Model of SLD (Case Study 2)")
plt.show()
