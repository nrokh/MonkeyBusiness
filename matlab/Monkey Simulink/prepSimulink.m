%% This file does most of the set up for the simulink model
% clear all;
% load HybridMonkeyDynamics1.mat;
% matlabFunction(ddq1, 'File', 'simulinkDynamics1');
% matlabFunction(ddq2, 'File', 'simulinkDynamics2');
% matlabFunction(ddq12, 'File', 'simulinkDynamics12');

%Read trajectory and process data
T = readmatrix('MonkeySwing.csv');

positions = timeseries(T(:, 6:9), T(:, 1));
velocities = timeseries(T(:, 13:16), T(:, 1));
torques = timeseries(T(2:end, 2:5), T(1:end-1, 1));
contact_modes = timeseries(T(:, 20), T(:,1));

% Create initial conditions from trajectory
q0_sim = T(1, 6:12);
dq0_sim = T(1, 13:19);