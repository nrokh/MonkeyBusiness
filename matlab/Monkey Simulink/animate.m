load HybridMonkeyDynamics1.mat;

disp('Animating...')

fig = figure();
g = struct();

g.gwe5 = matlabFunction(gwe5, 'vars', {q.'});
g.gws = matlabFunction(gws, 'vars', {q.'});
g.gwe1 = matlabFunction(gwe1, 'vars', {q.'});
g.gwe2 = matlabFunction(gwe2, 'vars', {q.'});
g.gwe3 = matlabFunction(gwe3, 'vars', {q.'});
g.gwe4 = matlabFunction(gwe4, 'vars', {q.'});

for i = 1:2:length(out.tout)-1
    a = tic;
    clf(fig);
    xlim([-1 1])
    ylim([-1.5 0.5])
%     axis equal
    xlabel('x')
    ylabel('y')
    set(gcf, 'DoubleBuffer', 'on');
    dt = out.tout(i) - out.tout(i+1);
    
    gwe5_h = g.gwe5(out.trajectory(i, 1:7));
    gws_h = g.gws(out.trajectory(i, 1:7));
    gwe1_h = g.gwe1(out.trajectory(i, 1:7)); 
    gwe2_h = g.gwe2(out.trajectory(i, 1:7));
    gwe3_h = g.gwe3(out.trajectory(i, 1:7)); 
    gwe4_h = g.gwe4(out.trajectory(i, 1:7));
    
    gwe5_t = g.gwe5(out.actual(i, 1:7));
    gws_t = g.gws(out.actual(i, 1:7));
    gwe1_t = g.gwe1(out.actual(i, 1:7)); 
    gwe2_t = g.gwe2(out.actual(i, 1:7));
    gwe3_t = g.gwe3(out.actual(i, 1:7)); 
    gwe4_t = g.gwe4(out.actual(i, 1:7));
    
    hold on
    
    plot([gwe5_t(1,3) gws_t(1,3)],[gwe5_t(2,3) gws_t(2,3)],'b-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    plot([gws_t(1,3) gwe1_t(1,3)],[gws_t(2,3) gwe1_t(2,3)],'k-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    plot([gws_t(1,3) gwe3_t(1,3)],[gws_t(2,3) gwe3_t(2,3)],'g-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    plot([gwe1_t(1,3) gwe2_t(1,3)],[gwe1_t(2,3) gwe2_t(2,3)],'m-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    plot([gwe3_t(1,3) gwe4_t(1,3)],[gwe3_t(2,3) gwe4_t(2,3)],'c-','LineWidth',3,'Marker','o','MarkerFaceColor','r')
    
    z = plot([gwe5_h(1,3) gws_h(1,3)],[gwe5_h(2,3) gws_h(2,3)],'b-','LineWidth',3,'Marker','o','MarkerFaceColor','r');
    z.Color(4) = 0.2;
    z = plot([gws_h(1,3) gwe1_h(1,3)],[gws_h(2,3) gwe1_h(2,3)],'k-','LineWidth',3,'Marker','o','MarkerFaceColor','r');
    z.Color(4) = 0.2;
    z = plot([gws_h(1,3) gwe3_h(1,3)],[gws_h(2,3) gwe3_h(2,3)],'g-','LineWidth',3,'Marker','o','MarkerFaceColor','r');
    z.Color(4) = 0.2;
    z = plot([gwe1_h(1,3) gwe2_h(1,3)],[gwe1_h(2,3) gwe2_h(2,3)],'m-','LineWidth',3,'Marker','o','MarkerFaceColor','r');
    z.Color(4) = 0.2;
    z = plot([gwe3_h(1,3) gwe4_h(1,3)],[gwe3_h(2,3) gwe4_h(2,3)],'c-','LineWidth',3,'Marker','o','MarkerFaceColor','r');
    z.Color(4) = 0.2;
    
    plot(0,0,'Marker','x','MarkerFaceColor','m','MarkerSize',30);
    plot(0.2, 0, 'Marker','x','MarkerFaceColor','m','MarkerSize',30);
    plot(0.4, 0, 'Marker','x','MarkerFaceColor','m','MarkerSize',30);
    
    hold off

    drawnow limitrate
    pause(dt - toc(a));
%     pause(0.01);
end

disp('Animation done.')