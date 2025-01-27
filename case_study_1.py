import numpy as np
import matplotlib.pyplot as plt



class PIController:
    def __init__(self, kp, ki, delta_t=1):
        self.kp = kp
        self.ki = ki
        self.delta_t = delta_t
        self.integral = 0
        self.feedforward=0

    def integral_action(self, setpoint,measured_value):
        # Update integral term
        error = setpoint - measured_value
        self.integral += error*self.delta_t 
        # Return PI action
        return self.kp * error + self.ki * self.integral


    def feedfoward_action(self, setpoint,measured_value,measured_disturbance):
        # Update integral term
        error = setpoint - measured_value
        self.integral += error*self.delta_t 
        # Return PI action
        uPI= self.kp * error + self.ki * self.integral
        # Calculate feedforward
        # Update feedforward term
        disturbance = measured_disturbance
        uFF=-(0.5)*disturbance 

        # Return PI+Feedforward action
        return uPI+1*uFF

    def calculate_max_pv_power(self, pcc_power,pv_power,nominal_pv_power,max_grid_allowed):
        load_observations=pcc_power-pv_power
        return  nominal_pv_power + self.integral_action(max_grid_allowed,pcc_power)
    
    def reset_integral(self):
        self.integral = 0
    
    def reset_feedforward(self):
        self.feedforward = 0

class Inverter:
    def __init__(self, nominal_power, delta_t=1, tau=2):
        self.pv_output_old = nominal_power
        self.pv_power = nominal_power
        self.alpha = delta_t / tau
        self.delta_t = delta_t
    
    def set_dynamic_model(self, delta_t,tau=2):
        self.delta_t = delta_t
        self.alpha = delta_t / tau



    def simulate_sld_discrete(self, pv_cmd_k):
        # Update PV power using discrete dynamic equation
        pv_next = (1 - self.alpha) * self.pv_output_old + self.alpha * pv_cmd_k

        # Calculate PCC power
        pv_output= self.pv_output_old
        self.pv_output_old= pv_next

        # Remove the initial state for plotting
        return pv_output



def simulate_case_study_1(max_grid_injection=1000, nominal_pv_power=1000, dt=1, steps=100):
    """
    Simulate PI control for Case Study 1.

    Parameters:
        pcc_power (float): Initial PCC power (W). 
        pv_output (float): Initial PV inverter power (W).
        max_grid_injection (float): Maximum allowed grid injection (W).
        nominal_pv_power (float): Nominal PV power (W).
        kp (float): Proportional gain for PI controller.
        ki (float): Integral gain for PI controller.
        dt (float): Time step for simulation (s).
        steps (int): Number of simulation steps.

    Returns:
        list: Time-series data of PCC power and PV commands.
    """

    # Initialize PI controller
    pi_controller = PIController(0.3, 0.3)
    inverter_pv = Inverter(1000)

    # Simulation parameters
    disturbance = [0] * 50 + [-100] * 50  # Constant disturbance (W)

    # Initialize variables
    pv_output = [1000]  # Initial PV power output (W)
    pcc_output = [1000]  # PCC power output
    pv_command=[]
    load_observations=[]


    for k in range(steps):
        print(max_grid_injection-pcc_output[-1])
        load_observations.append(pcc_output[-1]-pv_output[-1])
        pv_command.append(pi_controller.calculate_max_pv_power(pcc_output[-1], pv_output[-1], nominal_pv_power,max_grid_injection))
        pv_output.append(inverter_pv.simulate_sld_discrete(pv_command[-1]))
        pcc_output.append(disturbance[k]+pv_output[-1])

    return pv_output, pcc_output,pv_command,load_observations




# Simulation parameters
time_steps = 100
delta_t = 1.0  # Sampling time (s)
max_grid_injection=900

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



