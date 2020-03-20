clear;

vee = @(mat) [mat(1,3); mat(2, 3); mat(2,1)];

%% FORWARD KINEMATICS
disp("Starting forward kinematics...");
syms q_1 q_2 q_3 q_4 q_5 m_fore m_up m_tor l_fore l_up l_tor I_fore I_up I_tor real;
g_sb1 = [cos(q_1) sin(q_1) 0;
         -sin(q_1) cos(q_1) 0;
         0 0 1];
g_b1l1 = [1 0 0;
         0 1 -l_fore/2; 
         0 0 1];
g_l1e1 = [1 0 0;
          0 1 -l_fore/2;
          0 0 1];
g_e1b2 = [cos(q_2) sin(q_2) 0;
          -sin(q_2) cos(q_2) 0;
          0 0 1];
g_b2l2 = [1 0 0;
          0 1 -l_up/2;
          0 0 1];
g_l2e2 = [1 0 0;
          0 1 -l_up/2;
          0 0 1];
g_e2b3 = [cos(q_3) -sin(q_3) 0;
          sin(q_3) cos(q_3) 0;
          0 0 1];
g_b3l3 = [1 0 0;
          0 1 -l_tor/2;
          0 0 1];
g_l3e3 = [1 0 0;
          0 1 -l_tor/2;
          0 0 1];
g_b3b4 = [cos(q_4) -sin(q_4) 0;
          sin(q_4) cos(q_4) 0;
          0 0 1];
g_b4l4 = [1 0 0;
          0 1 l_up/2;
          0 0 1];
g_l4e4 = [1 0 0;
          0 1 l_up/2;
          0 0 1];
g_e4b5 = [cos(q_5) sin(q_5) 0;
          -sin(q_5) cos(q_5) 0;
          0 0 1];
g_b5l5 = [1 0 0;
          0 1 l_fore/2;
          0 0 1];
g_l5e5 = [1 0 0;
          0 1 l_fore/2;
          0 0 1];

g_sl1 = simplify(g_sb1*g_b1l1);
g_se1 = g_sl1*g_l1e1;
g_sb2 = g_se1*g_e1b2;
g_sl2 = simplify(g_sl1*g_l1e1*g_e1b2*g_b2l2);
g_se2 = g_sl2*g_l2e2;
g_sb3 = g_se2*g_e2b3;
g_sl3 = simplify(g_sl2*g_l2e2*g_e2b3*g_b3l3);
g_se3 = g_sl3*g_l3e3;
g_sb4 = g_sl2*g_l2e2*g_e2b3*g_b3b4;
g_sl4 = simplify(g_sl2*g_l2e2*g_e2b3*g_b3b4*g_b4l4);
g_se4 = g_sl4*g_l4e4;
g_sb5 = g_se4*g_e4b5;
g_sl5 = simplify(g_sl4*g_l4e4*g_e4b5*g_b5l5);
g_se5 = g_sl5*g_l5e5;

disp("Starting body jacobians...")
% for i = 1:5
%     eval(['Jb_sl' num2str(i) ' = [vee(g_sl' num2str(i) '\diff(g_sl' num2str(i) ', q_1)), vee(g_sl' ...
%         num2str(i) '\diff(g_sl' num2str(i) ', q_2)), vee(g_sl' num2str(i) ...
%         '\diff(g_sl' num2str(i) ', q_3)), vee(g_sl' num2str(i), '\diff(g_sl' ...
%         num2str(i) ', q_4)), vee(g_sl' num2str(i), '\diff(g_sl' num2str(i) ', q_5))];']);
% end

Jb_sl1 = [vee(inv(g_sl1)*diff(g_sl1, q_1)), vee(inv(g_sl1)*diff(g_sl1, q_2)), ...
    vee(inv(g_sl1)*diff(g_sl1, q_3)), vee(inv(g_sl1)*diff(g_sl1, q_4)), ...
    vee(inv(g_sl1)*diff(g_sl1, q_5))];
Jb_sl2 = [vee(inv(g_sl2)*diff(g_sl2, q_1)), vee(inv(g_sl2)*diff(g_sl2, q_2)), ...
    vee(inv(g_sl2)*diff(g_sl2, q_3)), vee(inv(g_sl2)*diff(g_sl2, q_4)), ...
    vee(inv(g_sl2)*diff(g_sl2, q_5))];
Jb_sl3 = [vee(inv(g_sl3)*diff(g_sl3, q_1)), vee(inv(g_sl3)*diff(g_sl3, q_2)), ...
    vee(inv(g_sl3)*diff(g_sl3, q_3)), vee(inv(g_sl3)*diff(g_sl3, q_4)), ...
    vee(inv(g_sl3)*diff(g_sl3, q_5))];
Jb_sl4 = [vee(inv(g_sl4)*diff(g_sl4, q_1)), vee(inv(g_sl4)*diff(g_sl4, q_2)), ...
    vee(inv(g_sl4)*diff(g_sl4, q_3)), vee(inv(g_sl4)*diff(g_sl4, q_4)), ...
    vee(inv(g_sl4)*diff(g_sl4, q_5))];
Jb_sl5 = [vee(inv(g_sl5)*diff(g_sl5, q_1)), vee(inv(g_sl5)*diff(g_sl5, q_2)), ...
    vee(inv(g_sl5)*diff(g_sl5, q_3)), vee(inv(g_sl5)*diff(g_sl5, q_4)), ...
    vee(inv(g_sl5)*diff(g_sl5, q_5))];

disp("Simplifying body jacobians...");
Jb_sl1 = simplify(Jb_sl1);
Jb_sl2 = simplify(Jb_sl2);
Jb_sl3 = simplify(Jb_sl3);
Jb_sl4 = simplify(Jb_sl4);
Jb_sl5 = simplify(Jb_sl5);
disp("Done simplifying body jacobians!");

%% MASS MATRIX M
disp("Starting M...");
M_up = [m_up 0 0; 0 m_up 0; 0 0 I_up];
M_fore = [m_fore 0 0; 0 m_fore 0; 0 0 I_fore];
M_tor = [m_tor 0 0; 0 m_tor 0; 0 0 I_tor];

M = Jb_sl1'*M_fore*Jb_sl1 + Jb_sl2'*M_up*Jb_sl2 + Jb_sl3'*M_tor*Jb_sl3 + Jb_sl4'*M_up*Jb_sl4 + Jb_sl5'*M_fore*Jb_sl5;
disp("Trying to simplify M...");
M = simplify(M);
disp("Done with M!");

%% COREOLIS MATRIX C
disp("Starting C...");
syms q_dot_1 q_dot_2 q_dot_3 q_dot_4 q_dot_5 real;
syms q_ddot_1 q_ddot_2 q_ddot_3 q_ddot_4 q_ddot_5 real;
q = [q_1; q_2; q_3; q_4; q_5];
q_dot = [q_dot_1; q_dot_2; q_dot_3; q_dot_4; q_dot_5];
q_ddot = [q_ddot_1; q_ddot_2; q_ddot_3; q_ddot_4; q_ddot_5];
C = cell(5,5);
for i = 1:5
    for j = 1:5
        sum = 0;
        for k = 1:5
            sum = sum + (diff(M(i,j), q(k)) + diff(M(i,k), q(j)) - diff(M(k,j), q(i)))*q_dot(k);
        end
        C(i,j) = {0.5*sum};
    end
end
C = cell2sym(C);
disp("Trying to simplify C...");
C = simplify(C);
disp("Done with C!");

%% POTENTIAL MATRIX N
disp("Starting with N...");
g = 9.81;
V = m_fore*g*g_sl1(2,3)+m_up*g*g_sl2(2,3)+m_tor*g*g_sl3(2,3)+m_up*g*g_sl4(2,3)+m_fore*g*g_sl5(2,3);
N = [diff(V, q_1); diff(V, q_2); diff(V, q_3); diff(V, q_4); diff(V, q_5)];
disp("Done with N!");

%% INPUT MATRIX UPSILON
disp("Starting with Y...");
syms tau_1 tau_2 tau_3 tau_4 real;
Y = [0; tau_1; tau_2; tau_3; tau_4];
disp("Done with Y!");

%% SUBSTITUTE CONSTANTS
syms2sub =  [l_fore, l_up, l_tor, m_fore, m_up, m_tor, I_fore,        I_up,          I_tor];
values   =  [0.2,    0.3,  0.1,   0.05,   0.05, 2.1,   0.2^2*0.05/12, 0.3^2*0.05/12, 0.1^2*2.1/12];

M = subs(M, syms2sub, values);
C = subs(C, syms2sub, values);
N = subs(N, syms2sub, values);

%% FULL DYNAMICS
disp("Computing full final dynamics");
dyn = simplify(M*q_ddot + C*q_dot + N - Y);
q_ddot_eqns = inv(M)*(Y - C*q_dot - N);
disp("Simplifying equations...");
q_ddot_eqns = simplify(q_ddot_eqns);
disp("Done!");

fid = fopen('MonkeyDynamics_qddot.txt', 'wt');
fprintf(fid, '%s\n\n\n', q_ddot_eqns(1));
fprintf(fid, '%s\n\n\n', q_ddot_eqns(2));
fprintf(fid, '%s\n\n\n', q_ddot_eqns(3));
fprintf(fid, '%s\n\n\n', q_ddot_eqns(4));
fprintf(fid, '%s\n\n\n', q_ddot_eqns(5));
fclose(fid);

fid = fopen('MonkeyDynamics.txt', 'wt');
fprintf(fid, '%s\n\n\n', dyn(1));
fprintf(fid, '%s\n\n\n', dyn(2));
fprintf(fid, '%s\n\n\n', dyn(3));
fprintf(fid, '%s\n\n\n', dyn(4));
fprintf(fid, '%s\n\n\n', dyn(5));
fclose(fid);