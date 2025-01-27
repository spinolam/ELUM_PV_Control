import numpy as np
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
    

class BESS:
    def __init__(self,  nominal_power, initial_soc=50, initial_voltage=0,delta_t=1, tau=2):
        self.soc = initial_soc
        self.voltage = initial_voltage
        self.bess_power = 0
        self.alpha = delta_t / tau
        self.delta_t = delta_t
        self.nominal_power = nominal_power
        self.nominal_capacity= 4.2 # Wh
        self.number_of_cells = 2


    def simulate_bess_discrete(self, current_power):
            # Parameters model Voc
            K1=0.1
            K2=1

            # Parameters model Resistance
            K3=0.25
            K4=0.04
            R0=0.005
            Voc_nom=8.4


            if self.soc < 95 and self.soc >5:

                soc_new = self.soc - self.delta_t*current_power*100/(3600*self.nominal_capacity)

                Rint=self.number_of_cells *(R0+K3/self.soc+K4/(100-self.soc))/2
                Eo=self.number_of_cells *(Voc_nom-K1*np.log(100-self.soc)-K2/self.soc)
                oc_voltage = Eo-(Rint)*current_power

                new_voltage = self.alpha*self.voltage + (1-self.alpha)*oc_voltage
            else:
                soc_new = self.soc 
                new_voltage = self.alpha*self.voltage 

            
            
            measured_soc = self.soc
            measured_voltage = self.voltage 
            measured_bess_power = current_power*measured_voltage
            self.soc = soc_new
            self.voltage  = new_voltage
            return measured_soc, measured_bess_power