fig = figure();
g = struct();
g.wp = matlabFunction(subs(g_wp, syms2sub, values), 'vars', {q'});
g.wb1 = matlabFunction(subs(g_wb1, syms2sub, values), 'vars', {q'});
g.we1 = matlabFunction(subs(g_we1, syms2sub, values), 'vars', {q'});
g.we2 = matlabFunction(subs(g_we2, syms2sub, values), 'vars', {q'});
p1 = [];
p2 = [];
p3 = [];

for i = 1:length(tout)-1
    a = tic;
    clf(fig);
    axis equal
    axis([-1 1 -2.5 1])
    xlabel('x')
    ylabel('y')
    set(gcf, 'DoubleBuffer', 'on');
    dt = tout(i) - tout(i+1);
    delete([p1, p2, p3]);
    g_wp_h = g.wp(xout(i, 1:5));
    g_wb1_h = g.wb1(xout(i, 1:5));
    g_we1_h = g.we1(xout(i, 1:5));
    g_we2_h = g.we2(xout(i, 1:5));
    hold on;
    plot([g_wp_h(1,3), g_wb1_h(1,3)], [g_wp_h(2,3), g_wb1_h(2,3)]);
    plot([g_wb1_h(1,3), g_we1_h(1,3)], [g_wb1_h(2,3), g_we1_h(2,3)]);
    plot([g_we1_h(1,3), g_we2_h(1,3)], [g_we1_h(2,3), g_we2_h(2,3)]);
    hold off
    drawnow limitrate
    pause(dt - toc(a));
end