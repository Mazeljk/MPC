Copyright 2018 The MathWorks, Inc.
************************************


DESCRIPTION
The folder contains the MATLAB and Simulink files that are used in the following Controls Tech Talk "Understanding Model Predictive Control, Part 7: Adaptive MPC Design With Simulink and Model Predictive Control Toolbox".

************************************
The folder contains the following items:
1-'Adaptive_MPC.slx': This is the Simulink model that is used to design and simulate the adaptive MPC controller.
2-'Meldas_library.slx': The custom library includes the blocks used in the 'Adaptive_MPC' Simulink model.
3-'Params.mat': This mat-file includes the parameters that are needed to run the Simulink model.
4-'car.jpg': This is the image used for the mask of the Plant block that can be found in the custom library.
5-'mask.jpg': This is the image used for the mask of the Reference block that can be found in the custom library.

*************************************
INSTRUCTION TO RUN THE SIMULINK MODEL
Double click the 'Params.mat' file to load all the parameters into the workspace. Open and run "Adaptive_MPC.slx" model.