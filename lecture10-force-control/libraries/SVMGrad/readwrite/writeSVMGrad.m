function  [] = writeSVMGrad(svmgrad, filename)
%WRITESVMGRAD writes an svmgrad object to a text file
% o svmgrad : svmgrad object
% o filename: filename for text file
%
% The text file will follow the same order of variables
%  model.D       : Datapoint Dimension
%  model.nSV     : Total # of Support Vectors
%  model.b       : Offset for classification function
%  model.sigma   : Gaussian RBF kernel Width
%  model.yalphas : Values for the Lagrangian multipliers*class  [1xnSV]
%  model.SVs     : Set of Support Vectors                       [DxnSV]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fileID = fopen(filename,'w');

% svmgrad.D
fprintf(fileID,'%d\n',svmgrad.D);
% svmgrad.nSV
fprintf(fileID,'%d\n',svmgrad.nSV);
% svmgrad.b
fprintf(fileID,'%4.8f\n',svmgrad.b);
% svmgrad.sigma
fprintf(fileID,'%4.8f\n\n',svmgrad.sigma);

% svmgrad.yalphas
for i=1:svmgrad.nSV
    fprintf(fileID,'%4.8f ',svmgrad.yalphas(i));
end
fprintf(fileID,'\n\n');

% svmgrad.SVs
for j=1:svmgrad.D
    for i=1:svmgrad.nSV
        fprintf(fileID,'%4.8f ',svmgrad.SVs(j,i));
    end
    fprintf(fileID,'\n');
end
fclose(fileID);

end

