syms q1 q2 dq1 dq2 tau1 tau2 real;

M = [3+2*cos(q2) 1+cos(q2); 1+cos(q2) 1];
C = [-dq2*sin(q2) -(dq1+dq2)*sin(q2); dq1*sin(q2) 0];
N = [20*cos(q1)+10*cos(q1+q2); 10*cos(q1+q2)];
Y = [tau1; tau2];
dq = [dq1; dq2];

ddq = inv(M)*(Y-C*dq-N);
ddq = simplify(ddq);

matlabFunction(ddq, 'File', 'pendulum_dyn');