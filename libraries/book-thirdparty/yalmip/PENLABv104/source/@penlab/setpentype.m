% Set index sets assigning constraint numbers to the type of constraint 
% penalization (which penalty function is used for each inequality). 
% By default, penalty-barrier and reciprocal penalty are used for function
% and matrix constraints, respectively. Constraints using log-barrier need
% to be explicitely named in *Ybar arrays.
function []=setpentype(obj,lbxbar,ubxbar,lbYbar,ubYbar)
  
  [ignore, indbar, ignore2] = intersect((obj.xboxmap.*obj.xboxmlt)',[-lbxbar,ubxbar]);
  obj.xboxindbar = indbar;
  obj.xboxindphi = setdiff([1:obj.Nxbox],indbar);
  %obj.xboxindbar = [1:obj.Nxbox];
  %obj.xboxindphi = [];

  %obj.ineqindbar = []; % will not use
  obj.ineqindphi = [1:obj.Nineq];

  [ignore, indbar, ignore2] = intersect((obj.Yboxmap.*obj.Yboxmlt)',[-lbYbar,ubYbar]);
  obj.Yboxindbar = indbar;
  obj.Yboxindphi = setdiff([1:obj.NYbox],indbar);
  %obj.Yboxindphi = [1:obj.NYbox];

  %obj.Aindbar = [];   % will not use at all 
  obj.Aindphi = [1:obj.NA];


