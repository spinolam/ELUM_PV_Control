class Inverter:
    def __init__(self, nominal_power, delta_t=1, tau=2):
        self.pv_output = nominal_power # initial state value
        self.alpha = delta_t / tau # 1st order dynamic
        self.delta_t = delta_t
    
    def set_dynamic_model(self, delta_t,tau=2):
        self.delta_t = delta_t
        self.alpha = delta_t / tau



    def simulate_sld_discrete(self, pv_cmd_k):
        # Update PV power using discrete dynamic equation
        pv_next = (1 - self.alpha) * self.pv_output + self.alpha * pv_cmd_k

        # Calculate PCC power
        pv_output= self.pv_output
        self.pv_output= pv_next

        # Remove the initial state for plotting
        return pv_output