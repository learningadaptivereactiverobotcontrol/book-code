function  solve_vts
%
% Solve the truss topology optimization problem by PENLAB
%
% Example: >> solve_tto('GEO/t3x3.geo')
%

load stiffmat2

par.A = A; par.IA = IA; par.nelem=nelem; par.nnod=nnod;

if nelem==16
    nx=4;ny=4;        %stiffmat1
elseif nelem==200
    nx=20;ny=10;      %stiffmat2
elseif nelem==420
    nx=30;ny=14;      %stiffmat3
elseif nelem==800
    nx=40;ny=20;      %stiffmat4
elseif nelem==1800
    nx=60;ny=30;      %stiffmat5
elseif nelem==3200
    nx=80;ny=40;      %stiffmat6
elseif nelem==5000
    nx=100;ny=50;      %stiffmat7
elseif nelem==7200
    nx=120;ny=60;      %stiffmat8    
else
    display('error in nelem');
end

par.nx = nx; par.ny = ny;

par.nloads = 1;
% par.ff{1} = zeros(par.n1,1); par.ff{1}(7) = -1;
% par.ff{2} = zeros(par.n1,1); par.ff{2}(11) = 1;

par.ff{1} = RHS;

problem = ttoml_define(par);
penm = penlab(problem);
solve(penm);

a2 = penm.x;
aa = reshape(a2(1:nelem),nx,ny);
imagesc(aa'); axis equal;

end

