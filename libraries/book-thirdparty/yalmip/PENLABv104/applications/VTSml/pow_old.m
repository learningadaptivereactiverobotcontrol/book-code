function  [lam,x] = pow( A,b )
%
% power iteration to find the largest eigenvalue of the inhomogeneous
% eigenvalue problem Ax - b = lam I x
%

n = length(b);
x = 1e10.*ones(n,1);
xo = ones(n,1);
eps = 1e-5;
err = 1;

k = 0;
while err>eps | k<2
    k = k + 1;
    y = A*x + b;
    lam = x'*y;
    x = y/norm(y);
    err = abs(x - xo);
    xo = x;
    if k>100, error('power method failed'), break, end
end
k;
x=(A-lam.*eye(n,n))\b; x=x/norm(x);

% Z=[zeros(2,2) eye(2,2);b*b'-A*A' A+A'];
% [aa,bb]=eig(Z);
% 
% k;