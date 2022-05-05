function [acc] =  my_accuracy(y_test, y_est)
%My_accuracy Computes the accuracy of a given classification estimate.
%   input -----------------------------------------------------------------
%   
%       o y_test  : (1 x M_test),  true labels from testing set
%       o y_est   : (1 x M_test),  estimated labes from testing set
%
%   output ----------------------------------------------------------------
%
%       o acc     : classifier accuracy
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% Implement Eq. X here %%%%
N = length(y_test);
N_errors = sum((y_test - y_est) ~= 0);
acc = (N - N_errors)/N;

end