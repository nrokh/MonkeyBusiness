comp = struct();
comp.computeM = matlabFunction(M, 'vars', {[q_1, q_2, x_o, y_o, theta_o]});
comp.computeC = matlabFunction(C, 'vars', {[q_1, q_2, x_o, y_o, theta_o], ...
    [q_dot_1, q_dot_2, x_dot_o, y_dot_o, theta_dot_o]});
comp.computeN = matlabFunction(N, 'vars', {[q_1, q_2, x_o, y_o, theta_o]});
comp.computeA = matlabFunction(A, 'vars', {[q_1, q_2, x_o, y_o, theta_o]});
comp.computeA_dot = matlabFunction(A_dot, 'vars', {[q_1, q_2, x_o, y_o, theta_o], ...
    [q_dot_1, q_dot_2, x_dot_o, y_dot_o, theta_dot_o]});

options = odeset();

q0 = [pi/2; -pi/2; -0.3; -0.45; 0];
dq0 = [0; 0; 0; 0; 0];
x0 = [q0;dq0];
tspan = [0 1];

[tout, xout] = ode45(@dynam, tspan, x0, options, comp);

function dx = dynam(t, x, comp)
q = x(1:5);
dq = x(6:10);
% A = comp.computeA(q');
% A_dot = comp.computeA_dot(q', dq');
M = comp.computeM(q');
C = comp.computeC(q', dq');
N = comp.computeN(q');
% lamb = inv(A*M*A')*(A*inv(M)*(-C*q-N)+A_dot*dq);
ddq = inv(M)*(-C*dq-N);
dx = [dq; ddq];
end