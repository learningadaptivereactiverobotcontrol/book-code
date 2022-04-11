function [ h ] = exp_loc_act(ls, c, x)
% Exponentially decaying function for 
    c_s = repmat(c,[1 size(x,1)])';
    diff_norm = sqrt(abs(sum((x - c_s).^2, 2)));    
    h = exp(- diff_norm.^2/ls);    
end

