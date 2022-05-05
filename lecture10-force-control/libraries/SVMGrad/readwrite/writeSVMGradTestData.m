function  [] = writeSVMGradTestData(x_test, y, value, gradient, filename)
%WRITESVMGRADTestData write testing data for the SVMGrad C++ implementation
% o x_test   : Dataset [DxM]
% o y        : labels  [1xM]
% o value    : labels  [1xM]
% o gradient : Dataset [DxM]
% o filename : -
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[D, M] = size(x_test);

fileID = fopen(filename,'w');

% D
fprintf(fileID,'%d\n',D);

% M
fprintf(fileID,'%d\n\n',M);

% x_test
for j=1:D
    for i=1:M
        fprintf(fileID,'%4.8f ',x_test(j,i));
    end
    fprintf(fileID,'\n');
end
fprintf(fileID,'\n');

% y
for i=1:M
    fprintf(fileID,'%d ',y(i));
end
fprintf(fileID,'\n\n');

% value
for i=1:M
    fprintf(fileID,'%4.8f ',value(i));
end
fprintf(fileID,'\n\n');

% gradient
for j=1:D
    for i=1:M
        fprintf(fileID,'%4.8f ',gradient(j,i));
    end
    fprintf(fileID,'\n');
end
fclose(fileID);

end

