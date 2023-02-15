function [c,ceq,dc,dceq] = toyCon(x)
% function [F,G] = toyusrfun(x)

c   = [];
dc = [];


ceq = [ x(1)   +   x(2)^2 + x(3)^2 - 2;
        x(2)^4 +   x(3)^4 + x(4)   - 4 ];
dceq = [ 1  2*x(2)     2*x(3)    0;
	 0  4*x(2)^3   4*x(3)^3  1 ];
