%% Set up problem
clc
clear all
close all

disp('Set up...')

% Helper funcs

vee = @(mat) [mat(1,3); mat(2,3); mat(2,1)];
adj = @(g) [[g(1,1) g(1,2);g(2,1) g(2,2)] [g(2,3);-g(1,3)]; zeros(1,2) 1];

% 2 arm monkey robot assuming fixed pin joint

syms q1 q2 q3 q4 x y p dq1 dq2 dq3 dq4 dx dy dp real

q = [q1; q2; q3; q4; x; y; p];
dq = [dq1; dq2; dq3; dq4; dx; dy; dp];

g = 9.81;

m1 = 0.38;
m2 = m1;
m3 = m1;
m4 = m1;
m5 = 2.1;

l1 = 0.3;
l2 = 0.3;
l3 = 0.3;
l4 = 0.3;
l5 = 0.3;

I1 = m1*l1^2/12;
I2 = m2*l2^2/12;
I3 = m3*l3^2/12;
I4 = m4*l4^2/12;
I5 = m5*l5^2/12;

lb1 = l1/2;
le1 = l1/2;
lb2 = l2/2;
le2 = l2/2;
lb3 = l3/2;
le3 = l3/2;
lb4 = l4/2;
le4 = l4/2;
lb5 = l5/2;
le5 = l5/2;

disp('Set up done.')

%% Transformations

disp('Transformations...')

gps = [1 0 0;
       0 1 lb5;
       0 0 1];
   
gsb1 = [cos(q1) -sin(q1) 0;
        sin(q1) cos(q1)  0;
        0       0        1];
    
gsb3 = [cos(q3) -sin(q3) 0;
        sin(q3) cos(q3)  0;
        0       0        1];
    
gb1l1 = [1 0 0;
         0 1 lb1;
         0 0 1];
          
gb3l3 = [1 0 0;
         0 1 lb3;
         0 0 1];
     
gl1e1 = [1 0 0;
         0 1 le1;
         0 0 1];
          
gl3e3 = [1 0 0;
         0 1 le3;
         0 0 1];

ge1b2 = [cos(q2) -sin(q2) 0;
         sin(q2) cos(q2)  0;
         0       0        1];
    
ge3b4 = [cos(q4) -sin(q4) 0;
         sin(q4) cos(q4)  0;
         0       0        1];

gb2l2 = [1 0 0;
         0 1 lb2;
         0 0 1];
          
gb4l4 = [1 0 0;
         0 1 lb4;
         0 0 1];
     
gl2e2 = [1 0 0;
         0 1 le2;
         0 0 1];
          
gl4e4 = [1 0 0;
         0 1 le4;
         0 0 1];
     
gpe5 = [1 0 0;
        0 1 -le5;
        0 0 1];
     
gwp = [cos(p) -sin(p) x;
        sin(p) cos(p) y;
        0       0     1];         

% For matrix calculations    

gpl1 = gps*gsb1*gb1l1;
gpl2 = gpl1*gl1e1*ge1b2*gb2l2;

gpl3 = gps*gsb3*gb3l3;
gpl4 = gpl3*gl3e3*ge3b4*gb4l4;

gwl1 = gwp*gpl1;
gwl2 = gwp*gpl2;

gwl3 = gwp*gpl3;
gwl4 = gwp*gpl4;

% For animation

gwe5 = gwp*gpe5;
gws = gwp*gps;
gwe1 = gwl1*gl1e1;
gwe2 = gwl2*gl2e2;
gwe3 = gwl3*gl3e3;
gwe4 = gwl4*gl4e4;
     
 disp('Transformations done.')
     
%% M Matrix

disp('M matrix...')

Jbpl1 = simplify([vee(inv(gpl1)*diff(gpl1,q1)) vee(inv(gpl1)*diff(gpl1,q2)) vee(inv(gpl1)*diff(gpl1,q3)) vee(inv(gpl1)*diff(gpl1,q4))]);
Jbpl2 = simplify([vee(inv(gpl2)*diff(gpl2,q1)) vee(inv(gpl2)*diff(gpl2,q2)) vee(inv(gpl2)*diff(gpl2,q3)) vee(inv(gpl2)*diff(gpl2,q4))]);
Jbpl3 = simplify([vee(inv(gpl3)*diff(gpl3,q1)) vee(inv(gpl3)*diff(gpl3,q2)) vee(inv(gpl3)*diff(gpl3,q3)) vee(inv(gpl3)*diff(gpl3,q4))]);
Jbpl4 = simplify([vee(inv(gpl4)*diff(gpl4,q1)) vee(inv(gpl4)*diff(gpl4,q2)) vee(inv(gpl4)*diff(gpl4,q3)) vee(inv(gpl4)*diff(gpl4,q4))]);

Jwp = simplify([vee(inv(gwp)*diff(gwp,x)) vee(inv(gwp)*diff(gwp,y)) vee(inv(gwp)*diff(gwp,p))]);

M1 = [m1 0  0;
      0  m1 0;
      0  0  I1];
  
M2 = [m2 0  0;
      0  m2 0;
      0  0  I2];
  
M3 = [m3 0  0;
      0  m3 0;
      0  0  I3];
  
M4 = [m4 0  0;
      0  m4 0;
      0  0  I4];
  
M5 = [m5 0  0;
      0  m5 0;
      0  0  I5];

M11 = simplify(Jbpl1.'*M1*Jbpl1 + Jbpl2.'*M2*Jbpl2 + Jbpl3.'*M3*Jbpl3 + Jbpl4.'*M4*Jbpl4);
M12 = simplify(Jbpl1.'*M1*adj(inv(gpl1))*Jwp + Jbpl2.'*M2*adj(inv(gpl2))*Jwp + Jbpl3.'*M3*adj(inv(gpl3))*Jwp + Jbpl4.'*M4*adj(inv(gpl4))*Jwp);
M21 = simplify(Jwp.'*adj(inv(gpl1)).'*M1*Jbpl1 + Jwp.'*adj(inv(gpl2)).'*M2*Jbpl2 + Jwp.'*adj(inv(gpl3)).'*M3*Jbpl3 + Jwp.'*adj(inv(gpl4)).'*M4*Jbpl4);
M22 = simplify(Jwp.'*adj(inv(gpl1)).'*M1*adj(inv(gpl1))*Jwp + Jwp.'*adj(inv(gpl2)).'*M2*adj(inv(gpl2))*Jwp + Jwp.'*adj(inv(gpl3)).'*M3*adj(inv(gpl3))*Jwp + Jwp.'*adj(inv(gpl4)).'*M4*adj(inv(gpl4))*Jwp + Jwp.'*M5*Jwp);

M = [M11 M12;
     M21 M22];

disp('M matrix done.')

%% C Matrix

disp('C matrix...')

C = sym(zeros(7,7));

for i = 1:7
    for j = 1:7
        for k = 1:7
            C(i,j) = C(i,j) + 0.5*(diff(M(i,j), q(k)) + diff(M(i,k), q(j)) - diff(M(k,j), q(i)))*dq(k);
        end
    end
end

C = simplify(C);

disp('C matrix done.')

%% N Matrix

disp('N matrix...')

V = m1*g*gwl1(2,3) + m2*g*gwl2(2,3) + m3*g*gwl3(2,3) + m4*g*gwl4(2,3) + m5*g*y; 

N = simplify([diff(V,q1); diff(V,q2); diff(V,q3); diff(V,q4); diff(V,x); diff(V,y); diff(V,p)]);
s
b = 0.1; 

N = N + [b*dq1; b*dq2; b*dq3; b*dq4; 0; 0; 0];

disp('N matrix done.')

%% A Matrix

disp('A matrices...')

a1 = gwe2(1:2,3) - 0;
a2 = [gwe4(2,3) - gwe4(1,3); gwe4(2,3) + gwe4(1,3)];

% a1 = gwe2(2,3) + 0;
% a2 = gwe4(2,3) + 0;

A1 = simplify(jacobian(a1, q));
A2 = simplify(jacobian(a2, q));

syms t q1t(t) q2t(t) q3t(t) q4t(t) xt(t) yt(t) pt(t)

qt = [q1t; q2t; q3t; q4t; xt; yt; pt];
dqt = diff(qt,t);

A1t = subs(A1, q, qt);
A2t = subs(A2, q, qt);

A1dt = diff(A1t, t);
A2dt = diff(A2t, t);

A1d = subs(A1dt, {[qt dqt]}, {[q dq]});
A2d = subs(A2dt, {[qt dqt]}, {[q dq]});

disp('A matrices done.')

%% Simulate

disp('Simulating...')

comp = struct();
comp.computeM = matlabFunction(M, 'vars', {q.'});
comp.computeC = matlabFunction(C, 'vars', {q.', dq.'});
comp.computeN = matlabFunction(N, 'vars', {q.', dq.'});
comp.computeA1 = matlabFunction(A1, 'vars', {q.', dq.'});
comp.computeA2 = matlabFunction(A2, 'vars', {q.', dq.'});
comp.computeA1d = matlabFunction(A1d, 'vars', {q.', dq.'});
comp.computeA2d = matlabFunction(A2d, 'vars', {q.', dq.'});

options = odeset('RelTol',1e-5,'AbsTol',1e-5);
% Both arms
% q0 = [-pi/4; pi/2; pi/4; -pi/2; 0; -l5/2 - l1*sqrt(2); 0]; 
% Arm 1
% q0 = [0; 0; pi/2; 0; 0; -l5/2 - l1 - l2; 0]; 
% Arm 2
% q0 = [pi/2; 0; 0; 0; 0; -l5/2 - l1 - l2; 0]; 

q0 = [-pi/2; pi/2; pi/2; -pi/2; -l1; -l5/2 - l2; 0];
dq0 = [0; 0; 0; 0; 0; 0; 0];
x0 = [q0;dq0];
tspan = [0 5];

[tout, xout] = ode45(@dynam, tspan, x0, options, comp);

disp('Simulation done.')

%% Animate

disp('Animating...')

fig = figure();
g = struct();

g.gwe5 = matlabFunction(gwe5, 'vars', {q.'});
g.gws = matlabFunction(gws, 'vars', {q.'});
g.gwe1 = matlabFunction(gwe1, 'vars', {q.'});
g.gwe2 = matlabFunction(gwe2, 'vars', {q.'});
g.gwe3 = matlabFunction(gwe3, 'vars', {q.'});
g.gwe4 = matlabFunction(gwe4, 'vars', {q.'});

for i = 1:1:length(tout)-1
    a = tic;
    clf(fig);
    xlim([-1 1])
    ylim([-1.5 0.5])
%     axis equal
    xlabel('x')
    ylabel('y')
    set(gcf, 'DoubleBuffer', 'on');
    dt = tout(i) - tout(i+1);
    
    gwe5_h = g.gwe5(xout(i, 1:7));
    gws_h = g.gws(xout(i, 1:7));
    gwe1_h = g.gwe1(xout(i, 1:7)); 
    gwe2_h = g.gwe2(xout(i, 1:7));
    gwe3_h = g.gwe3(xout(i, 1:7)); 
    gwe4_h = g.gwe4(xout(i, 1:7));
    
    hold on
    plot([gwe5_h(1,3) gws_h(1,3)],[gwe5_h(2,3) gws_h(2,3)],'b-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    plot([gws_h(1,3) gwe1_h(1,3)],[gws_h(2,3) gwe1_h(2,3)],'k-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    plot([gws_h(1,3) gwe3_h(1,3)],[gws_h(2,3) gwe3_h(2,3)],'g-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    plot([gwe1_h(1,3) gwe2_h(1,3)],[gwe1_h(2,3) gwe2_h(2,3)],'m-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    plot([gwe3_h(1,3) gwe4_h(1,3)],[gwe3_h(2,3) gwe4_h(2,3)],'c-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    
    plot(0,0,'Marker','x','MarkerFaceColor','m','MarkerSize',30)
    
    hold off
    drawnow limitrate
    pause(dt - toc(a));
%     pause(0.01);
end

disp('Animation done.')

%% Equations of Motion
% 
% syms lambda_1 lambda_2 real;
% lambda = [lambda_1; lambda_2];
% 
% disp("Computing full final dynamics");
% dyn = simplify(M*q_ddot + C*q_dot + N + A'*lambda - Y);
% q_ddot_eqns = inv(M)*(Y - C*q_dot - N - A'*lambda);
% disp("Simplifying equations...");
% q_ddot_eqns = simplify(q_ddot_eqns);
% disp("Done!");
% 
% fid = fopen('SMMonkeyDynamics_qddot.txt', 'wt');
% fprintf(fid, '%s\n\n\n', q_ddot_eqns(1));
% fprintf(fid, '%s\n\n\n', q_ddot_eqns(2));
% fprintf(fid, '%s\n\n\n', q_ddot_eqns(3));
% fprintf(fid, '%s\n\n\n', q_ddot_eqns(4));
% fprintf(fid, '%s\n\n\n', q_ddot_eqns(5));
% fclose(fid);
% 
% fid = fopen('SMMonkeyDynamics.txt', 'wt');
% fprintf(fid, '%s\n\n\n', dyn(1));
% fprintf(fid, '%s\n\n\n', dyn(2));
% fprintf(fid, '%s\n\n\n', dyn(3));
% fprintf(fid, '%s\n\n\n', dyn(4));
% fprintf(fid, '%s\n\n\n', dyn(5));
% fclose(fid);

%% Functions

function dx = dynam(t, x, comp)
q = x(1:7);
dq = x(8:14);
M = comp.computeM(q');
C = comp.computeC(q', dq');
N = comp.computeN(q', dq');
A1 = comp.computeA1(q', dq');
A2 = comp.computeA2(q', dq');
A1d = comp.computeA1d(q', dq');
A2d = comp.computeA2d(q', dq');

A = [A1];
Ad = [A1d];

% A = [A1; A2];
% Ad = [A1d; A2d];

[~, Mdag, Adag, ~] = BlockInverse(M,A);

ddq = Mdag*(-C*dq - N) - (Adag.'*Ad*dq);
% ddq = M\(-C*dq - N); % free fall
dx = [dq; ddq];
end

function [Binv, Mdag, Adag, Lam] = BlockInverse(M,A)
[mM,~] = size(M);
[mA,nA] = size(A);

block = [M A.';
        A zeros(mA,mA)];
Binv = inv(block);
Mdag = Binv(1:mM,1:mM);
Adag = Binv(mM+1:mM+mA,1:nA);
Lam = Binv(mM+1:mM+mA,mM+1:mM+mA);
end