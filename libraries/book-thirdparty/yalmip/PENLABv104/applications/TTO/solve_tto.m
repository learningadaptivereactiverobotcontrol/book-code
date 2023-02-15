function  solve_tto( iname )
%
% Solve the truss topology optimization problem by PENLAB
%
% Example: >> solve_tto('GEO/t3x3.geo')

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

par = kobum(iname);
problem = tto_define(par);
penm = penlab(problem);
solve(penm);
pic(par,penm.x);

end

