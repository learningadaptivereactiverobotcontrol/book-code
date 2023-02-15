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
moped = [255 255 255;
    255 247 255;
    252 232 255;
    247 217 255;
    241 204 255;
    231 189 255;
    221 176 255;
    209 166 255;
    194 153 255;
    178 143 255;
    161 133 255;
    145 122 255;
    124 112 255;
    105 105 255;
     97 110 255;
     89 117 255;
     82 122 255;
     74 131 255;
     69 143 255;
     64 156 255;
     56 169 255;
     51 180 255;
     48 196 255;
     43 213 255;
     38 230 255;
     36 248 255;
     31 255 248;
     28 255 229;
     26 255 209;
     23 255 189;
     20 255 169;
     18 255 152;
     15 255 131;
     13 255 110;
     13 255  89;
     10 255  67;
      8 255  49;
      8 255  28;
      8 255   8;
     26 255   5;
     47 255   5;
     63 255   5;
     83 255   3;
    104 255   3;
    125 255   3;
    146 255   3;
    162 255   3;
    183 255   0;
    204 255   0;
    225 255   0;
    246 255   0;
    255 247   0;
    255 225   0;
    255 204   0;
    255 183   0;
    255 162   0;
    255 145   0;
    255 123   0;
    255 102   0;
    255  81   0;
    255  60   0;
    255  42   0;
    255  21   0;
    255   0   0];
% rescale the colormap s.t. the values range from 0 to 1
moped = moped/255;
% % use the colormap
% colormap(moped)
% colorbar

