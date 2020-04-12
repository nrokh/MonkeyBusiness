%% This file does most of the set up for the simulink model

% matlabFunction(ddq1, 'File', 'simulinkDynamics');
m1 = 0.38;
m2 = 0.38;
m3 = m1;
m4 = m2;
m5 = 2.1;

%Read trajectory and process data


% Create initial conditions from trajectory
l1 = 0.3;
l2 = 0.3;
l3 = 0.3;
l4 = 0.3;
l5 = 0.3;
q0 = [0; 0; pi/2; 0; 0; -l5/2 - l1 - l2; 0];
dq0 = [0; 0; 0; 0; 0; 0; 0];