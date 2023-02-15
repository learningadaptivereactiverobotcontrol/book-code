function  solve_ttor
%
% Solve the robust truss topology optimization problem by PENLAB
%
% Example: >> solve_tto('GEO/t3x3.geo')
%

load stiffmat4

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

lb = 1;
for ie=1:nelem
    len = IA(ie);
    %clear ss;
    ss = sparse(A(lb:lb+len-1),A(lb+len:lb+2*len-1),A(lb+2*len:lb+3*len-1),nnod,nnod);
    hel = triu(ss) + tril(ss)' - diag(diag(ss));
    ss = hel +hel' - diag(diag(hel)) ;
    Kloc{ie} = ss;
    lb = lb + 3*len;
end


% 3x3ff_single
 nloads = 1;
% par.ff{1} = zeros(par.n1,1); par.ff{1}(9) =  10;
%par.ff{1} = zeros(par.n1,1); par.ff{1}(8) =  -10;
par.ff{1} = RHS;

% nloads = 1;
% I=find(RHS);
% par.ff{1} = zeros(nnod,1);par.ff{2} = zeros(nnod,1);
% par.ff{1}(2*nx-1) = 0;par.ff{1}(2*nx) = -.5;
% par.ff{1}(4*nx-1) = 0;par.ff{1}(4*nx) = -1;
% par.ff{1}(6*nx-1) = 0;par.ff{1}(6*nx) = -.5;
% par.ff{1}(2*nx-1) = 0;par.ff{1}(2*nx) = -.5;
% par.ff{1}(4*nx-1) = 0;par.ff{1}(2*nx-2) = -1;
% par.ff{1}(6*nx-1) = 0;par.ff{1}(2*nx-4) = -.5;
% par.ff{2}(nnod-4*nx-1) = 1;
% par.ff{2}(nnod-2*nx-1) = 2;
% par.ff{2}(nnod-1) = 1;
% par.ff{2}=1.5.*par.ff{2};


par.nloads = nloads;
nloadso = nloads;
for k=1:nloads
    normf(k) = norm(par.ff{k});
end
kappa = 0.3*min(normf);

problem = ttoml_define(par);
penm = penlab(problem); penm.opts.outlev=2;
solve(penm);
x = penm.x;

K = sparse(nnod,nnod);
for i=1:nelem
    K = K + x(i).*Kloc{i};
end

compl = x(nelem+1); compl0=compl;
%par.ff{1}'*(K\par.ff{1})

aa = reshape(penm.x(1:nelem),nx,ny);
imagesc(aa'); axis equal; pause(0.1);

%quiver(x1,x2,y1,y2,1.3,'-k','LineWidth',2,'MaxHeadSize',2.5); axis auto; pause(0.1);

str = sprintf( '   Iter     LC   rob    compl  compl0');
disp(str);
maxiter=10;lamb=[];
for iter = 1:maxiter
    for k = 1:nloadso
        I = find(par.ff{k});
        Ip=[];
        for i=1:length(I)
            nonu = ceil(I(i)/2);
            Ip = [Ip; nonu*2-1;nonu*2];
        end
        I = Ip;
        I = unique(I);
        J = setdiff(1:nnod,I);
        
        S = K(I,I) - K(I,J)*(K(J,J)\K(J,I));
        SI = inv(S);
        
        P = [1e-3 0; 0 kappa]; 
        if length(I)==6, P = blkdiag(P,P,P); end
        PI = inv(P);
        phi = atan(par.ff{k}(I(2))/par.ff{k}(I(1)));
        T = [cos(phi) sin(phi); -sin(phi) cos(phi)];
        if length(I)==6, T = blkdiag(T,T,T); end
        P = T'*P*T;
        [lam,gg] = pow(P'*SI*P,P'*SI*par.ff{k}(I));
        %lamb(k) = (kappa*kappa*lam/(2) + par.ff{k}(I)'*SI*par.ff{k}(I))/compliance;
        ggg = zeros(nnod,1);
        ggg(I) = P*gg;
        gh1 = par.ff{k} + ggg;
        gh2 = par.ff{k} - ggg;
        if (gh1(I))'*SI*gh1(I) > (gh2(I))'*SI*gh2(I)
            g{k} = par.ff{k} + ggg;
        else
            g{k} = par.ff{k} - ggg;
        end
        lamb(k) = (g{k}(I))'*SI*g{k}(I)/compl;
    end
    
    ilamb = lamb>1.05;
    nnloads = sum(ilamb);
    
    %lamb(ilamb)
    
    
    if nnloads == 0,
        for i=1:nloadso
            str = sprintf( ' %6d %6d %0.6g  ', iter, i, lamb(i));
            %disp(str);
            str = [str sprintf( ' %0.6g %0.6g  ', compl, compl0)];
            disp(str);
        end
        break
    end
    
    
    if iter>maxiter-1, break, end
    %quiver(x1,x2,y1,y2,0.3,'-r','LineWidth',2,'MaxHeadSize',2.5); axis auto; axis equal; pause(0.1);
    
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
    
    for i=1:nloadso
        if ilamb(i)==1
            str = sprintf( ' %6d %6d %0.6g  ', iter, i, lamb(i));
            %disp(str);
            
            str = [str sprintf( ' %0.6g %0.6g  ', compl, compl0)];
            disp(str);
        end
    end
    
    x = penm.x;
    compl = x(nelem+1);
    
    K = sparse(nnod,nnod);
    for i=1:nelem
        K = K + x(i).*Kloc{i};
    end
    compl0 = 0;
    for i=1:nloadso
        ccc = par.ff{i}'*(K\par.ff{i}); compl0 = max(ccc,compl0);
    end
    
    figure
aa = reshape(penm.x(1:nelem),nx,ny);
imagesc(aa'); axis equal;
    
end

% figure
% pic(par,penm.x);


for i=1:nloads
        fff=par.ff{i};
        %fdi = fff(abs(fff)>0);
        k=[1:nnod];
    str = sprintf('%0.6g ', i,k(abs(fff)>0),fff(abs(fff)>0)');
    %str = sprintf( ' Compliance   %0.6g , prescribed %0.6g ',coco,cmp );
    disp(str);
end

end

