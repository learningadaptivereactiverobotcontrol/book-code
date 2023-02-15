function mm = my_minmax(x)
%MINMAX Ranges of matrix rows.
x_sorted = sort(x, 'ascend');
mm = [x_sorted(1) x_sorted(end)];
