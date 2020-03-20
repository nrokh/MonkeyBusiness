%% Set up problem
clc
clear all
close all

disp('Set up...')

% Helper funcs

vee = @(mat) [mat(1,3); mat(2,3); mat(2,1)];
adj = @(g) [[g(1,1) g(1,2);g(2,1) g(2,2)] [g(2,3);-g(1,3)]; zeros(1,2) 1];

% 2 arm monkey robot assuming fixed pin joint

syms q1 q2 q3 dq1 dq2 dq3 real

q = [q1; q2; q3];
dq = [dq1; dq2; dq3];

g = 9.81;

m1 = 0.38;
m3 = m1;
m2 = 2.1;

l1 = 0.6;
l2 = 0.3;
l3 = 0.6;

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

gsb1 = [cos(q1) -sin(q1) 0;
        sin(q1) cos(q1)  0;
        0       0        1];

ge1b2 = [cos(q2) -sin(q2) 0;
         sin(q2) cos(q2)  0;
         0       0        1];

gb2e3 = [cos(q3) -sin(q3) 0;
         sin(q3) cos(q3)  0;
         0       0        1];

gb1l1 = [1 0 0;
         0 1 -lb1;
         0 0 1];
     
gb2l2 = [1 0 0;
         0 1 -lb2;
         0 0 1];
     
ge3l3 = [1 0 0;
         0 1 le3;
         0 0 1];
     
gl1e1 = [1 0 0;
         0 1 -le1;
         0 0 1];
     
gl2e2 = [1 0 0;
         0 1 -le2;
         0 0 1];
     
gl3b3 = [1 0 0;
         0 1 lb3;
         0 0 1];
     
% For M matrix     

gsl1 = gsb1*gb1l1;

gsl2 = gsl1*gl1e1*ge1b2*gb2l2;

gsl3 = gsl1*gl1e1*ge1b2*gb2e3*ge3l3;

% For animation

gse1 = gsl1*gl1e1;
gsb2 = gse1*ge1b2;
gse2 = gsb2*gb2l2*gl2e2;
gse3 = gsb2*gb2e3;
gsb3 = gse3*ge3l3*gl3b3;
     
 disp('Transformations done.')
     
%% M Matrix

disp('M matrix...')

Jbsl1 = simplify([vee(inv(gsl1)*diff(gsl1,q1)) vee(inv(gsl1)*diff(gsl1,q2)) vee(inv(gsl1)*diff(gsl1,q3))]);
Jbsl2 = simplify([vee(inv(gsl2)*diff(gsl2,q1)) vee(inv(gsl2)*diff(gsl2,q2)) vee(inv(gsl2)*diff(gsl2,q3))]);
Jbsl3 = simplify([vee(inv(gsl3)*diff(gsl3,q1)) vee(inv(gsl3)*diff(gsl3,q2)) vee(inv(gsl3)*diff(gsl3,q3))]);

M1 = [m1 0  0;
      0  m1 0;
      0  0  I1];
  
M2 = [m2 0  0;
      0  m2 0;
      0  0  I2];
  
M3 = [m3 0  0;
      0  m2 0;
      0  0  I3];

M = simplify(Jbsl1.'*M1*Jbsl1 + Jbsl2.'*M2*Jbsl2 + Jbsl3.'*M3*Jbsl3);

disp('M matrix done.')

%% C Matrix

disp('C matrix...')

C = sym(zeros(3,3));

for i = 1:3
    for j = 1:3
        for k = 1:3
            C(i,j) = C(i,j) + 0.5*(diff(M(i,j), q(k)) + diff(M(i,k), q(j)) - diff(M(k,j), q(i)))*dq(k);
        end
    end
end

C = simplify(C);

disp('C matrix done.')

%% N Matrix

disp('N matrix...')

V = m1*g*gsl1(2,3) + m2*g*gsl2(2,3) + m3*g*gsl3(2,3); 

N = simplify([diff(V,q1); diff(V,q2); diff(V,q3)]);

b = 1;

N = N + [0; b*dq2; b*dq3];

disp('N matrix done.')

%% Simulate

disp('Simulating...')

comp = struct();
comp.computeM = matlabFunction(M, 'vars', {q.'});
comp.computeC = matlabFunction(C, 'vars', {q.', dq.'});
comp.computeN = matlabFunction(N, 'vars', {q.', dq.'});

options = odeset();

q0 = [0; 0; pi/2];
dq0 = [0; 0; 0];
x0 = [q0;dq0];
tspan = [0 10];

[tout, xout] = ode45(@dynam, tspan, x0, options, comp);

disp('Simulation done.')

%% Animate

disp('Animating...')

fig = figure();
g = struct();

g.sb2 = matlabFunction(gsb2, 'vars', {q.'});
g.se2 = matlabFunction(gse2, 'vars', {q.'});
g.sb3 = matlabFunction(gsb3, 'vars', {q.'});

for i = 1:length(tout)-1
    a = tic;
    clf(fig);
    xlim([-1 1])
    ylim([-2 0])
%     axis equal
    xlabel('x')
    ylabel('y')
    set(gcf, 'DoubleBuffer', 'on');
    dt = tout(i) - tout(i+1);
    
    b1x = 0;
    b1y = 0;
    
    gsb2_h = g.sb2(xout(i, 1:3));
    gse2_h = g.se2(xout(i, 1:3));
    gsb3_h = g.sb3(xout(i, 1:3));
    
    hold on
    plot([b1x gsb2_h(1,3)],[b1y gsb2_h(2,3)],'b-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    plot([gsb2_h(1,3) gse2_h(1,3)],[gsb2_h(2,3) gse2_h(2,3)],'b-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    plot([gsb2_h(1,3) gsb3_h(1,3)],[gsb2_h(2,3) gsb3_h(2,3)],'b-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    
    hold off
    drawnow limitrate
    pause(dt - toc(a));
end

disp('Animation done.')

%% Functions

function dx = dynam(t, x, comp)
q = x(1:3);
dq = x(4:6);
M = comp.computeM(q');
C = comp.computeC(q', dq');
N = comp.computeN(q', dq');
ddq = M\(-C*dq-N);
dx = [dq; ddq];
end