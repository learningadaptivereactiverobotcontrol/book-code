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
% par.ff{1} = zeros(par.n1,1); par.ff{1}(9) =  10;
%par.ff{1} = zeros(par.n1,1); par.ff{1}(8) =  -10;
% %3x3ff
% nloads = 2;
% par.ff{1} = zeros(par.n1,1); par.ff{1}(7) =  10;
% par.ff{2} = zeros(par.n1,1); par.ff{2}(11) = 10;
% % 5x5ff
% nloads = 3;
% par.ff{1} = zeros(par.n1,1); par.ff{1}(31) = 7; par.ff{1}(32) =  -7;
% par.ff{2} = zeros(par.n1,1); par.ff{2}(39) = 10;
% par.ff{3} = zeros(par.n1,1); par.ff{3}(26) = 10;
% nloads = 5;
% par.ff{1} = zeros(par.n1,1); par.ff{1}(31) = 10; 
% par.ff{2} = zeros(par.n1,1); par.ff{2}(33) = 10;
% par.ff{3} = zeros(par.n1,1); par.ff{3}(35) = 10;
% par.ff{4} = zeros(par.n1,1); par.ff{4}(37) = 10;
% par.ff{5} = zeros(par.n1,1); par.ff{5}(39) = 10;
%  nloads = 1;
% % par.nloads = nloads;
 par.ff{1} = zeros(par.n1,1); %par.ff{1}(31) =  10;
 par.ff{1}(40) =  -10;
% 7x3br_ff
% nloads = 5;
% par.ff{1} = zeros(par.n1,1); par.ff{1}(6) = -10; 
% par.ff{2} = zeros(par.n1,1); par.ff{2}(12) = -10;
% par.ff{3} = zeros(par.n1,1); par.ff{3}(18) = -10;
% par.ff{4} = zeros(par.n1,1); par.ff{4}(24) = -10;
% par.ff{5} = zeros(par.n1,1); par.ff{5}(30) = -10;
% 7x7
% nloads = 3;
% par.ff{1} = zeros(par.n1,1); par.ff{1}(71) =  10;
% par.ff{2} = zeros(par.n1,1); par.ff{2}(83) = 10;
% par.ff{3} = zeros(par.n1,1); par.ff{3}(50) = -10;
% 11x5
% nloads = 1;
% par.ff{1} = par.f;


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

compl = x(m+1); compl0=compl;
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
        J = setdiff(1:n1,I);
        
        S = K(I,I) - K(I,J)*(K(J,J)\K(J,I));
        SI = inv(S);
        
        P = [1e-3 0; 0 kappa]; PI = inv(P);
        if length(I)==4, P = blkdiag(P,P); end
        phi = atan(par.ff{k}(I(2))/par.ff{k}(I(1)));
        T = [cos(phi) sin(phi); -sin(phi) cos(phi)];
        if length(I)==4, T = blkdiag(T,T); end
        P = T'*P*T;
        [lam,gg] = pow(P'*SI*P,P'*SI*par.ff{k}(I));
        %lamb(k) = (kappa*kappa*lam/(2) + par.ff{k}(I)'*SI*par.ff{k}(I))/compliance;
        ggg = zeros(n1,1);
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
    
    x1=[];x2=[];y1=[];y2=[];
    for i=1:nloadso
        if ilamb(i)==1
            fff = g{i};
            
            I = find(fff);
            nonu = ceil(I(1)/2);
            I = [nonu*2-1;nonu*2];
            for j=1:length(I)/2
                x1(i) = xym(I(j+1)/2,1); x2(i)=xym(I(j+1)/2,2);
                y1(i) = fff(I(j))/(10*norm(fff)); y2(i) = fff(I(j+1))/(10*norm(fff));
            end
        end
    end
    
    if iter>maxiter-1, break, end
    quiver(x1,x2,y1,y2,1.3,'-r','LineWidth',2,'MaxHeadSize',2.5);  pause(0.1);
    
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
    compl = x(m+1);
    
    K = sparse(n1,n1);
    for i=1:m
        K = K + x(i).*Kloc{i};
    end
    compl0 = 0;
    for i=1:nloadso
        ccc = par.ff{i}'*(K\par.ff{i}); compl0 = max(ccc,compl0);
    end
    
    figure
    pic(par,penm.x);%axis auto;axis image;
    
end

% figure
% pic(par,penm.x);


for i=1:nloads
        fff=par.ff{i};
        %fdi = fff(abs(fff)>0);
        k=[1:n1];
    str = sprintf('%0.6g ', i,k(abs(fff)>0),fff(abs(fff)>0)');
    %str = sprintf( ' Compliance   %0.6g , prescribed %0.6g ',coco,cmp );
    disp(str);
end

end

