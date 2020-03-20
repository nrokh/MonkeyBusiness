%% Set up problem
clc
clear all
close all

disp('Set up...')

% Helper funcs

vee = @(mat) [mat(1,3); mat(2,3); mat(2,1)];
adj = @(g) [[g(1,1) g(1,2);g(2,1) g(2,2)] [g(2,3);-g(1,3)]; zeros(1,2) 1];

% 2 arm monkey robot assuming fixed pin joint

syms q1 q2 x y p dq1 dq2 dx dy dp real

q = [q1; q2; x; y; p];
dq = [dq1; dq2; dx; dy; dp];

g = 9.81;

m1 = 0.38;
m2 = m1;
m3 = 2.1;

l1 = 0.6;
l2 = 0.6;
l3 = 0.3;

I1 = m1*l1^2/12;
I2 = m2*l2^2/12;
I3 = m3*l3^2/12;

lb1 = l1/2;
le1 = l1/2;
lb2 = l2/2;
le2 = l2/2;
lb3 = l3/2;
le3 = l3/2;

disp('Set up done.')

%% Transformations

disp('Transformations...')

gps = [1 0 0;
       0 1 lb3;
       0 0 1];
   
gsb1 = [cos(q1) -sin(q1) 0;
        sin(q1) cos(q1)  0;
        0       0        1];
    
gsb2 = [cos(q2) -sin(q2) 0;
        sin(q2) cos(q2)  0;
        0       0        1];
    
gb1l1 = [1 0 0;
         0 1 lb1;
         0 0 1];
          
gb2l2 = [1 0 0;
         0 1 lb2;
         0 0 1];
     
gl1e1 = [1 0 0;
         0 1 le1;
         0 0 1];
          
gl2e2 = [1 0 0;
         0 1 le2;
         0 0 1];
     
gpe3 = [1 0 0;
        0 1 -le3;
        0 0 1];
     
gwp = [cos(p) -sin(p) x;
        sin(p) cos(p) y;
        0       0     1];         

% For matrix calculations    

gpl1 = gps*gsb1*gb1l1;
gpl2 = gps*gsb2*gb2l2;

gwl1 = gwp*gpl1;
gwl2 = gwp*gpl2;

% For animation

gwe3 = gwp*gpe3;
gws = gwp*gps;
gwe1 = gwl1*gl1e1;
gwe2 = gwl2*gl2e2;
     
 disp('Transformations done.')
     
%% M Matrix

disp('M matrix...')

Jbpl1 = simplify([vee(inv(gpl1)*diff(gpl1,q1)) vee(inv(gpl1)*diff(gpl1,q2))]);
Jbpl2 = simplify([vee(inv(gpl2)*diff(gpl2,q1)) vee(inv(gpl2)*diff(gpl2,q2))]);

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

M11 = simplify(Jbpl1.'*M1*Jbpl1 + Jbpl2.'*M2*Jbpl2);
M12 = simplify(Jbpl1.'*M1*adj(inv(gpl1))*Jwp + Jbpl2.'*M2*adj(inv(gpl2))*Jwp);
M21 = simplify(Jwp.'*adj(inv(gpl1)).'*M1*Jbpl1 + Jwp.'*adj(inv(gpl2)).'*M2*Jbpl2);
M22 = simplify(Jwp.'*adj(inv(gpl1)).'*M1*adj(inv(gpl1))*Jwp + Jwp.'*adj(inv(gpl2)).'*M2*adj(inv(gpl2))*Jwp + Jwp.'*M3*Jwp);

M = [M11 M12;
     M21 M22];

disp('M matrix done.')

%% C Matrix

disp('C matrix...')

C = sym(zeros(5,5));

for i = 1:5
    for j = 1:5
        for k = 1:5
            C(i,j) = C(i,j) + 0.5*(diff(M(i,j), q(k)) + diff(M(i,k), q(j)) - diff(M(k,j), q(i)))*dq(k);
        end
    end
end

C = simplify(C);

disp('C matrix done.')

%% N Matrix

disp('N matrix...')

V = m1*g*gwl1(2,3) + m2*g*gwl2(2,3) + m3*g*y; 

N = simplify([diff(V,q1); diff(V,q2); diff(V,x); diff(V,y); diff(V,p)]);

b = 0.5; 

N = N + [b*dq1; b*dq2; 0; 0; 0];

disp('N matrix done.')

%% A Matrix

disp('A matrices...')

a1 = gwe1(1:2,3) + 0;
a2 = gwe2(1:2,3) + 0;

A1 = jacobian(a1, q);
A2 = jacobian(a2, q);

syms t q1t(t) q2t(t) xt(t) yt(t) pt(t)

qt = [q1t; q2t; xt; yt; pt];
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

options = odeset();

q0 = [0; pi/2; 0; -l1 - l3/2; 0];
dq0 = [0; 0; 0; 0; 0];
x0 = [q0;dq0];
tspan = [0 10];

[tout, xout] = ode45(@dynam, tspan, x0, options, comp);

disp('Simulation done.')

%% Animate

disp('Animating...')

fig = figure();
g = struct();

g.gwe3 = matlabFunction(gwe3, 'vars', {q.'});
g.gws = matlabFunction(gws, 'vars', {q.'});
g.gwe1 = matlabFunction(gwe1, 'vars', {q.'});
g.gwe2 = matlabFunction(gwe2, 'vars', {q.'});

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
    
    gwe3_h = g.gwe3(xout(i, 1:5));
    gws_h = g.gws(xout(i, 1:5));
    gwe1_h = g.gwe1(xout(i, 1:5)); 
    gwe2_h = g.gwe2(xout(i, 1:5)); 
    
    hold on
    plot([gwe3_h(1,3) gws_h(1,3)],[gwe3_h(2,3) gws_h(2,3)],'b-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    plot([gws_h(1,3) gwe1_h(1,3)],[gws_h(2,3) gwe1_h(2,3)],'k-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    plot([gws_h(1,3) gwe2_h(1,3)],[gws_h(2,3) gwe2_h(2,3)],'g-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    
    plot(0,0,'Marker','x','MarkerFaceColor','m','MarkerSize',30)
    
    hold off
    drawnow limitrate
    pause(dt - toc(a));
%     pause(0.01);
end

disp('Animation done.')

%% Functions

function dx = dynam(t, x, comp)
q = x(1:5);
dq = x(6:10);
M = comp.computeM(q');
C = comp.computeC(q', dq');
N = comp.computeN(q', dq');
A1 = comp.computeA1(q', dq');
A2 = comp.computeA2(q', dq');
A1d = comp.computeA1d(q', dq');
A2d = comp.computeA2d(q', dq');

A = [A1];
Ad = [A1d];

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