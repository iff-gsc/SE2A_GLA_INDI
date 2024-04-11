# Boosted Incremental Nonlinear Dynamic Inversion for Intense Flexible Airplane Gust Load Alleviation

This repository contains the source code used for the research results in [1].
It provides a simulation environment for active gust load alleviation for an aeroelastic airplane implemented in Matlab/Simulink.

<div align="center">
<h3>Video Summary</h3>
  <a href="https://youtu.be/WXsBBEiZN1M">
    <img 
      src="https://img.youtube.com/vi/WXsBBEiZN1M/0.jpg" 
      style="width:50%;">
  </a>
</div>

[1] Beyer, Y., Steen, M., & Hecker, P. (2024). Boosted Incremental Nonlinear Dynamic Inversion for Flexible Airplane Gust Load Alleviation. _Journal of Guidance, Control, and Dynamics_.



## Installation

- MATLAB: You need MATLAB/Simulink 2018b or later. You do not need any additional toolboxes.
- Clone project including the submodules:
  ```
  git clone --recursive https://github.com/iff-gsc/SE2A_GLA_INDI.git
  ```
- TiXI (optional; for changing aircraft configuration): You need to install TiXI 2.2.3: https://github.com/DLR-SC/tixi
- TiGL (optional; for changing aircraft configuration): You need to install TiGL 2.2.3: https://github.com/DLR-SC/tigl
- FlexiFlightVis 0.2 (optional; for visualization): https://github.com/iff-gsc/FlexiFlightVis


## Demo

- Initialize the simulation model:  
  1. Open MATLAB.
  2. Navigate to the project folder and then to the subfolder [demo](demo):
     ```
     cd('demo')
     ```
  3. Run the initialization script [init_flexible_unsteady_indi](demo/init_flexible_unsteady_indi.m) (Click `Change Folder` or `Add to Path` if requested.):
     ```
     init_flexible_unsteady_indi
     ```
- Run the Simulink simulation:
  1. The Simulink model [sim_flexible_unsteady_indi](models/sim_flexible_unsteady_indi.slx) should already open during the initialization.
  2. Run the Simulink model.
  3. Several signals are logged in the `Scopes` subsystem.
- Run FlexiFlightVis to see what happens.


## Reproduction of the Diagrams of the Paper

The `plot_scripts` folder contains Matlab scripts that can be used to reproduce the diagrams from the paper.
The table below shows which scripts can be used to create which diagrams.
All scripts work independently.

Figure | Filename
--- | --- 
1 | `plot_block_diagram_pics.m` 
3 | `plot_gusts.m` 
9 | `plot_actuator_boost_example.m` 
10 | `plot_booster_avoid_noise.m` 
11 | `plot_booster_avoid_noise.m` 
14 | `plot_mass_distribution.m` 
15a | `plot_wing_definition.m`
15b | `plot_control_effectiveness.m` 
16 | `plot_eigenmodes.m` 
18a | `plot_wrbm_mode_contributions.m` 
18b | `plot_wrbm_mode_contributions_gusts.m` 
19 | `plot_gust_response.m` 
20 | `plot_gust_response_delay.m` 
21 | `plot_gust_response_control_inputs.m` 
22 | `plot_gust_response_delay_gain.m` 
23 | `plot_gust_envelope_delay_rate_limit.m` 
24 | `plot_gust_envelope_delay_rate_limit.m` 
25 | `plot_gust_response_robust_servo.m` 
26 | `plot_gust_response_robust_delay.m` 
27 | `plot_gust_response_robust_cntrl_effect.m` 
28 | `plot_gust_response_failure.m` 


The plot scripts mostly consist of an initialization of the parameters with trim calculation, a simulation, the creation of the figures and the export to a TikZ file.
The export to a TikZ file is deactivated by default and can be activated by setting: `is_tikz_export_desired = true`.
The trim calculation takes quite a long time.
Since the same trim point is often used in different plot scripts, you can often skip the initialization with trim calculation after it has been executed for the first time.
In addition, the very first trim calculation takes a very long time because the Simulink model has to be compiled.
FlexiFlightVis can be used for visualization during the simulations.

Unfortunately, there may be slight differences between the paper and this repository for some figures, as minor changes were made after the final paper was submitted.

Information about the aeroelastic flight dynamics model can be found in this repository: https://github.com/iff-gsc/SE2A_Aviation_2023