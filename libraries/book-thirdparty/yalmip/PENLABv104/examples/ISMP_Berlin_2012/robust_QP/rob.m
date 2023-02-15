% A=[7 -1 0 0; -1 2 -1 0; 0 -1 2 -1; 0 0 -1 1];
n=9;A=rand(n,n);A=A*A';%+eye(n,n);
b=[1.3; .8; 1; 2];
b=rand(n,1);
unc = 0.1;
n=length(A);

pen = qps(A,b);   % nominal QP
p0 = penlab(pen); p0.opts.outlev=0;
solve(p0); x0 = p0.x;

pen=qps(A+unc*eye(n,n),b);   % robust counterpart
p1 = penlab(pen); p1.opts.outlev=0;
solve(p1); x1=p1.x;

pen=qps(A-unc*eye(n,n),b);   % robust counterpart
p1a = penlab(pen); p1a.opts.outlev=0;
solve(p1a); x1a=p1a.x;

q1=x1'*(A+unc*eye(n,n))*x1-b'*x1+2;
q1a=x1a'*(A-unc*eye(n,n))*x1a-b'*x1a+2;

xx = x1;
if q1a > q1, xx = x1a; end

pen=rob_sdpsub(A,b,unc,xx); % find feasible A
p2 = penlab(pen); p2.opts.outlev=0;
solve(p2); U = p2.Y{1};

pen = qps(A+unc.*U,b); %robust feasible counterpart
p3 = penlab(pen); p3.opts.outlev=0;
solve(p3); x2 = p3.x;


q0=x0'*A*x0-b'*x0+2;
q1=max(q1,q1a);
q2=x2'*(A+unc*U)*x2-b'*x2+2;
fprintf('   nominal   robust infeas.  robust feas.\n')
fprintf('   %6.4f    %6.4f          %6.4f     \n',q0,q1,q2)

