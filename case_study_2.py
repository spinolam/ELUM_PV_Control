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
    

class BESS:
    def __init__(self, nominal_power, delta_t=1, tau=2):
        self.bess_output_old = nominal_power
        self.bess_power = nominal_power
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


def calculate_max_pv_power(self, pcc_power,pv_power, bess_power, bess_soc, nominal_pv_power, nominal_bess_power,max_grid_allowed):
        #load_observations=pcc_power-pv_power -bess_power


        
        maximum_pv_power_command = nominal_pv_power+integral_action(max_grid_allowed,pcc_power)
        bess_power_setpoint = nominal_bess_power + proportial_action(max_grid_allowed,pcc_power)


        # Enforce SOC constraints
        if bess_soc >= soc_max and bess_power_setpoint<0:
            bess_power_setpoint=0# Prevent charging
        elif bess_soc <= soc_min and bess_power_setpoint>0:
            bess_power_setpoint=0# Prevent discharging

            
        return  bess_power_setpoint, maximum_pv_power_command