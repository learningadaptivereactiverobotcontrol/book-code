%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                                                          %
%% mopedcolor()                                             %
%%                                                          %
%% generates the moped colormap for use with MATLAB         %
%%                                                          %
%% Usage: move file mopedcolor.m in MATLAB search path      %
%% Then use the command lines:                              %
%%                                                          %
%% moped = mopedcolor;                                      %
%% colormap(moped)                                          %
%% colorbar                                                 %
%%                                                          %
%% Written by Steffi Gaile                                  %
%% Date: 2008/04/08 11:23                                   %
%%                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function moped = mopedcolor()

% % Here the origin of the color values is explained
% % This is according to the file colo.f by A. Diaz, C. Soto and M. Kocvara
% 
% % Define the Nr of color steps
% StepNr = 63;
% % partition the interval [0,1] into StepNr intervals
% dens = [0 : 1/StepNr : 1]';
% % calculate hue (taking values from 0 to 0.84)
% hue = 0.84*(1 - dens);
% % calculate saturation (taking values from 0 to 1)
% sat = max(0, (1 - (hue./0.833).^4));
% % calculate brightness (taking values from 0 to 1)
% bright = ones(length(hue),1);
% % rescale the above values s.t. 
% % hue between 0 and 360 degrees
% % saturation between 0 and 100 percent
% % brightness between 0 and 100 percent
% hue = 360*hue;
% sat = 100*sat;
% bright = 100*bright;
% % collect the values in the matrix hsb_color
% hsb_color = [hue , sat , bright];

% the obtained colorvalues in the hsb color scheme have
% the following values in the rgb color scheme
m1 = [255:-1:0]';
moped = [m1,m1,m1];
% rescale the colormap s.t. the values range from 0 to 1
moped = moped/255;
% % use the colormap
% colormap(moped)
% colorbar

