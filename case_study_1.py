import numpy as np
import matplotlib.pyplot as plt
from control_tools.controllers import PIController
from control_tools.processes import Inverter

class PVInverterMaxController():

    def __init__(self, kp, ki, delta_t=1):
        # Call the parent class's initializer
        self.controller_pv= PIController(kp, ki, delta_t)
        
    def calculate_controller_output(self, pcc_power_out,pv_power_out,nominal_pv_power,max_grid_allowed,max_pv_power_lim=np.inf):
        # Observe load disturbance (necessary if feedforward)
        load_observations=pcc_power_out-pv_power_out
        # Find new max allowed pv power with integral corrections
        new_max_pv_power = nominal_pv_power + self.controller_pv.integral_action(max_grid_allowed,pcc_power_out)
        # Saturate allowed pv power nominal value <=PV_max <= max_value_possible
        new_max_pv_power=max(nominal_pv_power, min(new_max_pv_power, max_pv_power_lim)) #uncomment this line to use saturation
        return  new_max_pv_power
    

def simulate_case_study_1(max_grid_injection=1000, nominal_pv_power=1000, dt=1, steps=100):
    """
    Simulate PI control for Case Study 1.

    Parameters:
        max_grid_injection (float): Maximum grid injection power (W).
        nominal_pv_power (float): Nominal PV power (W).
        dt (float): Sampling time (s).
        steps (int): Number of simulation steps.   
    Returns:
        list: Time-series data of PCC power, PV power, PV command and load observations.
    """

    # Initialize PI controller
    pi_controller = PVInverterMaxController(0.3, 0.3)
    inverter_pv = Inverter(1000)

    # Simulation parameters
    disturbance = [0] * 50 + [-100] * 50  # Constant disturbance (W)

    # Initialize variables
    pv_output = [1000]  # Initial PV power output (W)
    pcc_output = [1000]  # PCC power output
    pv_command=[]
    load_observations=[]


    for k in range(steps):
 
        load_observations.append(pcc_output[-1]-pv_output[-1])
        pv_command.append(pi_controller.calculate_controller_output(pcc_output[-1], pv_output[-1], nominal_pv_power,max_grid_injection))
        pv_output.append(inverter_pv.simulate_sld_discrete(pv_command[-1]))
        pcc_output.append(disturbance[k]+pv_output[-1])

    return pv_output, pcc_output,pv_command,load_observations




# Simulation parameters
time_steps = 100
delta_t = 1.0  # Sampling time (s)
max_grid_injection=1000

# Run the simulation
pv_output, pcc_output, pv_command,load_observations = simulate_case_study_1(max_grid_injection=max_grid_injection)

# Plot results
time = np.arange(0, time_steps * delta_t, delta_t)
plt.figure(figsize=(10, 6))
plt.plot(time, pv_output[:-1], label="PV Output (W)")
plt.plot(time, pcc_output[:-1], label="PCC Output (W)")
plt.plot(time, pv_command[:], label="PV Command (W)")
plt.plot(time,load_observations[:], label="Loads (W)")
plt.axhline(max_grid_injection, color="red", linestyle="--", label="Max Grid Injection (W)")
plt.xlabel("Time (s)")
plt.ylabel("Power (W)")
plt.title("Discrete Dynamic Model of SLD (Case Study 1)")
plt.legend()
plt.grid()
plt.show()



