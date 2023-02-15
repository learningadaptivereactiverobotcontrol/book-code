function  [lam,x] = pow( A,b )
%
% power iteration to find the largest eigenvalue of the inhomogeneous
% eigenvalue problem Ax - b = lam I x
%

n = length(b);
Ab=A*b;

Z=[zeros(n,n) eye(n,n);b*b'-A*A' A+A'];
[aa,bb]=eig(Z);

k=0;
for i=1:2*n
    if isreal(bb(i,i))
        k = k+1;
        l(k) = bb(i,i);
        x{k}=(A-l(k).*eye(n,n))\Ab; x{k}=x{k}/norm(x{k});
        ff(k)=(x{k}-b)'*A*(x{k}-b);
    end
end
        
[maxl,mind] = max(ff);
lam = l(mind);
x=x{mind};