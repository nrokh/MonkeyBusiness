clear;
vee = @(mat) [mat(1,3); mat(2,3); mat(2,1)];
adj = @(g) [[g(1,1) g(1,2);g(2,1) g(2,2)] [g(2,3);-g(1,3)]; zeros(1,2) 1];

%% Forward Kinematics
disp("Starting forward kinematics...");
syms x_o y_o theta_o q_1 q_2 q_3 q_4 m_fore m_up m_tor l_fore l_up l_tor I_fore I_up I_tor real;

q_l = [q_1; q_2]; % States of the links
q_o =  [x_o; y_o; theta_o]; % States of the object
q = [q_l; q_o]; % Combined state

g_wp = [cos(theta_o) -sin(theta_o) x_o;
        sin(theta_o) cos(theta_o) y_o;
        0 0 1]; 
Jb_wp = [vee(inv(g_wp)*diff(g_wp, x_o)), vee(inv(g_wp)*diff(g_wp, y_o)), ...
    vee(inv(g_wp)*diff(g_wp, theta_o))];

g_ps = [1 0 0;
        0 1 l_tor/2;
        0 0 1];
g_sb1 = [cos(q_1) -sin(q_1) 0;
         sin(q_1) cos(q_1) 0;
         0 0 1];
g_b1l1 = [1 0 l_up/2;
          0 1 0;
          0 0 1];
g_l1e1 = g_b1l1;
g_e1b2 = [cos(q_2) -sin(q_2) 0;
          sin(q_2) cos(q_2) 0;
          0 0 1];
g_b2l2 = [1 0 l_fore/2;
          0 1 0;
          0 0 1];
g_l2e2 = g_b2l2;

g_pl1 = g_ps*g_sb1*g_b1l1;
g_pl2 = g_pl1*g_l1e1*g_e1b2*g_b2l2;
g_wl1 = g_wp*g_pl1;
g_wl2 = g_wp*g_pl2;

g_wb1 = g_wp*g_ps*g_sb1;
g_we1 = g_wb1*g_b1l1*g_l1e1;
g_we2 = g_we1*g_e1b2*g_b2l2*g_l2e2;

Jb_pl1 = [vee(inv(g_pl1)*diff(g_pl1, q_1)), vee(inv(g_pl1)*diff(g_pl1, q_2))];
Jb_pl2 = [vee(inv(g_pl2)*diff(g_pl2, q_1)), vee(inv(g_pl2)*diff(g_pl2, q_2))];

Jb_wp = simplify(Jb_wp);
Jb_pl1 = simplify(Jb_pl1);
Jb_pl2 = simplify(Jb_pl2);

%% Mass Matrix M
disp("Starting M...")
M_tor = [m_tor 0 0; 0 m_tor 0; 0 0 I_tor];
M_up = [m_up 0 0; 0 m_up 0; 0 0 I_up];
M_fore = [m_fore 0 0; 0 m_fore 0; 0 0 I_fore];

M11 = Jb_pl1'*M_up*Jb_pl1 + Jb_pl2'*M_fore*Jb_pl2;
M12 = Jb_pl1'*M_up*inv(adj(g_pl1))*Jb_wp + Jb_pl2'*M_fore*inv(adj(g_pl2))*Jb_wp;
M21 = Jb_wp'*inv(adj(g_pl1))'*M_up*Jb_pl1 + Jb_wp'*inv(adj(g_pl2))'*M_fore*Jb_pl2;
M22 = Jb_wp'*M_tor*Jb_wp + Jb_wp'*inv(adj(g_pl1))'*M_up*inv(adj(g_pl1))*Jb_wp + Jb_wp'*inv(adj(g_pl2))'*M_fore*inv(adj(g_pl2))*Jb_wp;

disp("Simplifying M...")
M = [simplify(M11) simplify(M12); simplify(M21) simplify(M22)];

%% Coreolis Matrix C
disp("Starting C...")
syms q_dot_1 q_dot_2 x_dot_o y_dot_o theta_dot_o real;
syms q_ddot_1 q_ddot_2 x_ddot_o y_ddot_o theta_ddot_o real;

q_dot = [q_dot_1; q_dot_2; x_dot_o; y_dot_o; theta_dot_o];
q_ddot = [q_ddot_1; q_ddot_2; x_ddot_o; y_ddot_o; theta_ddot_o];
C  = sym(zeros(5,5));
for ii = 1:5
    for jj = 1:5
        for kk = 1:5
            C(ii,jj) = C(ii,jj) + 1/2*(diff(M(ii,jj),q(kk)) + diff(M(ii,kk),q(jj)) - diff(M(kk,jj),q(ii)))*q_dot(kk);
        end
    end
end
disp("Simplifying C...")
C = simplify(C);

%% Nonlinear Matrix N
disp("Starting N...")
g = 9.81;
V = m_tor*g*y_o + m_up*g*g_wl1(2,3) + m_fore*g*g_wl2(2,3);
N = jacobian(V, q)';
disp("Simplifying N...")
N = simplify(N);

%% Applied Forces Matrix Y
disp("Starting Y...");
syms tau_1 tau_2 real;
Y = [tau_1; tau_2; 0; 0; 0];

%% Constraint Matrix A
disp("Starting A...")
Bc1 = [1 0; 0 1; 0 0];
g_wo = [cos(theta_o) -sin(theta_o) x_o;
        sin(theta_o) cos(theta_o) y_o;
        0 0 1];
g_wc1 = [1 0 0;
        0 1 0;
        0 0 1];
g_oc1 = simplify(inv(g_wo)*g_wc1);
Gs = -adj(inv(g_oc1))'*Bc1;
Jb_op = [cos(theta_o) sin(theta_o) 0
    -sin(theta_o) cos(theta_o) 0
    0 0 1];
GsbarT = Gs'*Jb_op;

g_so = [1 0 0;
        0 1 -l_tor/2;
        0 0 1];
g_sc1 = g_so*g_oc1;
g_se2 = g_sb1*g_b1l1*g_l1e1*g_e1b2*g_b2l2*g_l2e2;
Js_se2 = [vee(diff(g_se2, q_1)*inv(g_se2)), vee(diff(g_se2, q_2)*inv(g_se2))];
Jh = Bc1'*adj(inv(g_sc1))*Js_se2;
A = [-Jh GsbarT];
disp("Simplifying A...")
A = simplify(A);

%% Compute A_dot
disp("Starting A_dot...")
syms q1t(t) q2t(t) xot(t) yot(t) thetaot(t) real;
A_sub = subs(A, {q_1, q_2, x_o, y_o, theta_o}, {q1t, q2t, xot, yot, thetaot});
A_dot = diff(A_sub, t);
A_dot = subs(A_dot, {q1t, q2t, xot, yot, thetaot, diff(q1t,t), diff(q2t,t),...
    diff(xot,t), diff(yot,t), diff(thetaot,t)}, {q_1, q_2, x_o, y_o, theta_o, ...
    q_dot_1, q_dot_2, x_dot_o, y_dot_o, theta_dot_o});
disp("Simplifying A_dot...")
A_dot = simplify(A_dot);

%% Substitute Constants
disp("Substituting constants...")
syms2sub =  [l_fore, l_up, l_tor, m_fore, m_up, m_tor, I_fore,        I_up,          I_tor];
values   =  [0.3,    0.3,  0.3,   0.38,   0.38, 2.1,   0.3^2*0.38/12, 0.3^2*0.38/12, 0.3^2*2.1/12];

M = subs(M, syms2sub, values);
C = subs(C, syms2sub, values);
N = subs(N, syms2sub, values);
A = subs(A, syms2sub, values);
A_dot = subs(A, syms2sub, values);

%% Full Dynamics EOMs
syms lambda_1 lambda_2 real;
lambda = [lambda_1; lambda_2];

disp("Computing full final dynamics");
dyn = simplify(M*q_ddot + C*q_dot + N + A'*lambda - Y);
q_ddot_eqns = inv(M)*(Y - C*q_dot - N - A'*lambda);
disp("Simplifying equations...");
q_ddot_eqns = simplify(q_ddot_eqns);
disp("Done!");

fid = fopen('SMMonkeyDynamics_qddot.txt', 'wt');
fprintf(fid, '%s\n\n\n', q_ddot_eqns(1));
fprintf(fid, '%s\n\n\n', q_ddot_eqns(2));
fprintf(fid, '%s\n\n\n', q_ddot_eqns(3));
fprintf(fid, '%s\n\n\n', q_ddot_eqns(4));
fprintf(fid, '%s\n\n\n', q_ddot_eqns(5));
fclose(fid);

fid = fopen('SMMonkeyDynamics.txt', 'wt');
fprintf(fid, '%s\n\n\n', dyn(1));
fprintf(fid, '%s\n\n\n', dyn(2));
fprintf(fid, '%s\n\n\n', dyn(3));
fprintf(fid, '%s\n\n\n', dyn(4));
fprintf(fid, '%s\n\n\n', dyn(5));
fclose(fid);