# Code with the resolution of ELUM Recruitment Process Challenge
This repository contains the resolution of the ELUM recruitment process, which consists of a coding problem for two case studies. The resolution of the case study 1 is in the file `case_study_1.py` and case study 2 is in the file `case_study.2.py`.

Falta:
1. Figuras control loops
1.1 Objetivos
4. Possiveis amelhoramentos
5. Resultados 


1. Comentatios
2. arquivo requirements.txt
2.2 arquivo .git_ignore
3. 1 teste de validacao


## Case study 1


**Objective:** Implement a controller algorithm whose output must be calculated so to optimize the PV production while respecting:
> The active power injected into the grid should not exceed the control parameter Maximum active power injection into the grid where 
pcc_power = pv_power_out + load_power

*Inputs*:
pcc_power (variable)
pv_power_out (variable)
max_power_injection_grid (parameter)
nominal_pv_power (parameter)

*Output*:
max_pv_power_command (variable)

**Closed-loop system**

*Control objectives*
1. Optimize PV production
2. Respect max_power_injection_grid>= pcc_power

*Assumptions*:
1. PV inverter is an inner loop control with an optimal controller
2. We assume PV inverter has a dynamic of a first-order response
3. We assume load is only negative
4. Loads are random

**Strategy**
>Guarantee that any load in the system can compensated by PV production

**Proposition** : 
1. An PI controller that minimizes the error between pcc_power and maximum of the grid 
2. It can be enough to rejection of lead disturbances
3. Add of a saturation to guarantee max_pv_power_command respects nominal operation


**Results**
**Possible improvements** :
1. Include feedforward actions if loads as we can estimate loads
2. Model PV inverter dynamics to find control gain with optimization methods



## Case study 2


**Objective:** Implement a controller algorithm whose output must be calculated so to optimize the PV production while respecting:
> The active power injected into the grid should not exceed the control parameter Maximum active power injection into the grid where 
pcc_power = pv_power_out + load_power + bess_power_out

*Inputs*:
pcc_power (variable)
pv_power_out (variable)
bess_power_out (variable)
bess_soc (variable)
max_power_injection_grid (parameter)
nominal_pv_power (parameter)
nominal_bess_power (parameter)

*Outputs*:
max_pv_power_command (variable)
bess_power_setpoit (variable)

**Closed-loop system**

*Control objectives*
1. Optimize PV production
2. Respect max_power_injection_grid>= pcc_power

*Assumptions*:
1. PV inverter is an inner loop control with an optimal controller
2. We assume PV inverter has a dynamic of a first-order response
3. We assume load is only negative
4. Loads are random
5. BESS can be reduced to a elecrical court-circuit model 
6. BESS has a minimum and maximum power to respect
7. BESS charges or discharges up to max/min capacity

**Strategy**
Prioritize PV inverter for negative loads
Use battery to compensate PV inverter saturation
Use battery to store PV inverter excess output
Battery PI slower than PV inverter to prioritize PV inverter



