function [g,userdata] = sdp_confun(x,Y,userdata)
% returns all inequalities at once, expect g(x)<=0
% vector Ng x 1

 g=sum(x(1:end-1));


