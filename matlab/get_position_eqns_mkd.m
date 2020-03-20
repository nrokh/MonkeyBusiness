fid = fopen('MonkeyPositions.txt', 'wt');

x_b = cell(5,1);
y_b = cell(5,1);
x_e = cell(5,1);
y_e = cell(5,1);

for i = 1:5
    eval(['x_b(i) = {g_sb' num2str(i) '(1, 3)};']);
    fprintf(fid, 'x_b%s = %s\n', num2str(i), cell2sym(x_b(i)));
    eval(['y_b(i) = {g_sb' num2str(i) '(2, 3)};']);
    fprintf(fid, 'y_b%s = %s\n', num2str(i), cell2sym(y_b(i)));
    eval(['x_e(i) = {g_se' num2str(i) '(1, 3)};']);
    fprintf(fid, 'x_e%s = %s\n', num2str(i), cell2sym(x_e(i)));
    eval(['y_e(i) = {g_se' num2str(i) '(2, 3)};']);
    fprintf(fid, 'y_e%s = %s\n', num2str(i), cell2sym(y_e(i)));
end

fclose(fid);