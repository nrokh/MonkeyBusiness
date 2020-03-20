comp = struct();
comp.computeM = matlabFunction(M, 'vars', {q});
comp.computeC = matlabFunction(C, 'vars', {q, q_dot});
comp.computeN = matlabFunction(N, 'vars', {q});

options = odeset();

q0 = [-pi/12; -pi/12; pi/12; pi/12; pi/12];
dq0 = [0; 0; 0; 0; 0];
x0 = [q0;dq0];
tspan = [0 2];

[tout, xout] = ode45(@dynam, tspan, x0, options, comp);

function dx = dynam(t, x, comp)
q = x(1:5);
dq = x(6:10);
M = comp.computeM(q);
C = comp.computeC(q, dq);
N = comp.computeN(q);
ddq = M\(-C*dq-N);
dx = [dq; ddq];
end