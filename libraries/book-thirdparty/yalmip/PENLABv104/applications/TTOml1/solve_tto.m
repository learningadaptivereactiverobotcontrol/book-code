function  solve_tto( iname )
%
% Solve the truss topology optimization problem by PENLAB
%
% Example: >> solve_tto('GEO/t3x3.geo')
%

par = kobum(iname);

par.nloads = 2;
par.ff{1} = zeros(par.n1,1); par.ff{1}(7) = -1;
par.ff{2} = zeros(par.n1,1); par.ff{2}(11) = 1;

problem = ttoml_define(par);
penm = penlab(problem);
solve(penm);
pic(par,penm.x);

end

