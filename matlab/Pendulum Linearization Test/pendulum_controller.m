% Double Pendulum trajectory

M = readmatrix('DoublePendulumSwing.csv');
posn = timeseries(M(:, 4:5), M(:, 1));
torques = timeseries(M(2:end, 2:3), M(1:end-1, 1));