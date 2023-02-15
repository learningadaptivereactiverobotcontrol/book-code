function  solve_ttor( iname )
%
% Solve the robust truss topology optimization problem by PENLAB
%
% Example: >> solve_tto('GEO/t3x3.geo')
%

par = kobum(iname);

m=par.m; n=par.n; n1=par.n1; BI=par.BI; xy=par.xy;
maska=par.maska; ijk=par.ijk;

len = zeros(m,1);
for i=1:m
    x1=xy(ijk(i,2)/2,1); y1=xy(ijk(i,2)/2,2);
    x2=xy(ijk(i,4)/2,1); y2=xy(ijk(i,4)/2,2);
    len(i,1)=sqrt((x1-x2)^2 + (y1-y2)^2);
end

for i=1:m
    Ahelp = len(i)*BI(i,:)'*BI(i,:);
    Kloc{i} = Ahelp(maska,maska);
end

% 3x3ff_single
nloads = 1;
par.ff{1} = zeros(par.n1,1); par.ff{1}(9) =  10;
% 3x3ff
% nloads = 2;
% par.ff{1} = zeros(par.n1,1); par.ff{1}(7) =  10;
% par.ff{2} = zeros(par.n1,1); par.ff{2}(11) = 10;
% 5x5ff
% nloads = 3;
% par.ff{1} = zeros(par.n1,1); par.ff{1}(31) = 10; par.ff{1}(32) =  -10;
% par.ff{2} = zeros(par.n1,1); par.ff{2}(39) = 10;
% par.ff{3} = zeros(par.n1,1); par.ff{3}(26) = 10;
% % nloads = 1;
% par.nloads = nloads;
% par.ff{1} = zeros(par.n1,1); par.ff{1}(31) =  10;par.ff{1}(32) =  -10;
% 7x7
% nloads = 3;
% par.ff{1} = zeros(par.n1,1); par.ff{1}(71) =  10;
% par.ff{2} = zeros(par.n1,1); par.ff{2}(83) = 10;
% par.ff{3} = zeros(par.n1,1); par.ff{3}(50) = 5;

par.nloads = nloads;
nloadso = nloads;
for k=1:nloads
    normf(k) = norm(par.ff{k});
end
kappa = 0.3*min(normf);

problem = ttoml_define(par);
penm = penlab(problem); penm.opts.outlev=0;
solve(penm);
x = penm.x;

K = sparse(n1,n1);
for i=1:m
    K = K + x(i).*Kloc{i};
end

compliance = x(m+1)
%par.ff{1}'*(K\par.ff{1})

pic(par,penm.x); pause(0.1);

xym = xy(maska(2:2:end)/2,:);
for i=1:nloads
    fff = par.ff{i};
    I = find(fff);
    nonu = ceil(I(1)/2);
    I = [nonu*2-1;nonu*2];
    
    for j=1:length(I)/2
        x1(i) = xym(I(j+1)/2,1); x2(i)=xym(I(j+1)/2,2);
        y1(i) = fff(I(j))/(10*norm(fff)); y2(i) = fff(I(j+1))/(10*norm(fff));
    end
end
quiver(x1,x2,y1,y2,1.3,'-k','LineWidth',2,'MaxHeadSize',2.5); axis auto; pause(0.1);

maxiter=4;
for iter = 1:maxiter
    for k = 1:nloadso
        I = find(par.ff{k});
        nonu = ceil(I(1)/2);
        I = [nonu*2-1;nonu*2];
        J = setdiff(1:n1,I);
        
        S = K(I,I) - K(I,J)*(K(J,J)\K(J,I));
        SI = inv(S);
        
        [lam,gg] = pow1(SI);
        %lamb(k) = (kappa*kappa*lam/(2) + par.ff{k}(I)'*SI*par.ff{k}(I))/compliance;
        ggg = zeros(n1,1);
        ggg(I) = kappa.*gg;
        g{k} = ggg;
        lamb(k) = (g{k}(I))'*SI*g{k}(I)/compliance;
    end
    
    ilamb = lamb>1.05;
    nnloads = sum(ilamb);
    
    max(lamb)
    lamb(ilamb)
    
    for i=1:nnloads
        if ilamb==1
            fff = g{i};
        end
        I = find(fff);
        nonu = ceil(I(1)/2);
        I = [nonu*2-1;nonu*2];
        for j=1:length(I)/2
            x1(i) = xym(I(j+1)/2,1); x2(i)=xym(I(j+1)/2,2);
            y1(i) = fff(I(j))/(10*norm(fff)); y2(i) = fff(I(j+1))/(10*norm(fff));
        end
    end
   
    
    if iter>maxiter-1, break, end
    quiver(x1,x2,y1,y2,1.3,'-r','LineWidth',2,'MaxHeadSize',2.5); axis auto; axis equal; pause(0.1);
    
    iii=1;
    for ii=1:nloadso
        if ilamb(ii)
            par.ff{nloads+iii} = g{ii};
            iii = iii+1;
        end
    end
    nloads = nloads + nnloads;
    par.nloads = nloads;
    
    problem = ttoml_define(par);
    penm = penlab(problem); penm.opts.outlev=0;
    solve(penm);
    
    x = penm.x;
    compliance = x(m+1)
    
    K = sparse(n1,n1);
    for i=1:m
        K = K + x(i).*Kloc{i};
    end
    
    figure
pic(par,penm.x);axis auto
    
end

% figure
% pic(par,penm.x);

for i=1:nloads
    fff=par.ff{i};
    fff(abs(fff)>0)
end

end

