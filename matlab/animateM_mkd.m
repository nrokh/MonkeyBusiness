fig = figure();
calc = struct();
calc.sb1 = matlabFunction(subs(g_sb1, syms2sub, values), 'vars', {q'});
calc.sb2 = matlabFunction(subs(g_sb2, syms2sub, values), 'vars', {q'});
calc.sb3 = matlabFunction(subs(g_sb3, syms2sub, values), 'vars', {q'});
calc.se3 = matlabFunction(subs(g_se3, syms2sub, values), 'vars', {q'});
calc.sb5 = matlabFunction(subs(g_sb5, syms2sub, values), 'vars', {q'});
calc.se5 = matlabFunction(subs(g_se5, syms2sub, values), 'vars', {q'});

bRecord = 0;  % Modify this to save a video
if bRecord
    % Define video recording parameters
    Filename = 'monkeysim';
    v = VideoWriter(Filename, 'MPEG-4');
    myVideo.Quality = 100;
    open(v);
end

for i = 1:length(tout)-1
    a = tic;
    clf(fig);
    xlim([-0.5 0.5])
    ylim([-1 0])
%     axis equal
    xlabel('x')
    ylabel('y')
    set(gcf, 'DoubleBuffer', 'on');
    dt = tout(i) - tout(i+1);
    g_sb1_h = calc.sb1(xout(i, 1:5));
    g_sb2_h = calc.sb2(xout(i, 1:5));
    g_sb3_h = calc.sb3(xout(i, 1:5));
    g_se3_h = calc.se3(xout(i, 1:5));
    g_sb5_h = calc.sb5(xout(i, 1:5));
    g_se5_h = calc.se5(xout(i, 1:5));
    hold on;
    plot([g_sb1_h(1,3), g_sb2_h(1,3)], [g_sb1_h(2,3), g_sb2_h(2,3)]);
    plot([g_sb2_h(1,3), g_sb3_h(1,3)], [g_sb2_h(2,3), g_sb3_h(2,3)]);
    plot([g_sb3_h(1,3), g_se3_h(1,3)], [g_sb3_h(2,3), g_se3_h(2,3)]);
    plot([g_sb3_h(1,3), g_sb5_h(1,3)], [g_sb3_h(2,3), g_sb5_h(2,3)]);
    plot([g_sb5_h(1,3), g_se5_h(1,3)], [g_sb5_h(2,3), g_se5_h(2,3)]);
    hold off
    drawnow limitrate
    if bRecord
        frame = getframe(gcf);
        writeVideo(v,frame);
    else
        pause(dt - toc(a)); % waits if drawing frame took less time than anticipated
    end
end

if bRecord
    close(v);
end