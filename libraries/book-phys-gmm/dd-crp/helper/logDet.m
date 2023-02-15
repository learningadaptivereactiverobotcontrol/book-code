function logdet = logDet (Sigma)
%     logdet = 2*sum( log( diag( chol( Sigma ) ) ) );
    
    [L, U, P] = lu(Sigma);
    du = diag(U);
    c = det(P) * prod(sign(du));
    logdet = log(c) + sum(log(abs(du)));
    
end

%   v = logdet(A);
%       computes the logarithm of determinant of A. 
%
%       Here, A should be a square matrix of double or single class.
%       If A is singular, it will returns -inf.
%
%       Theoretically, this function should be functionally 
%       equivalent to log(det(A)). However, it avoids the 
%       overflow/underflow problems that are likely to 
%       happen when applying det to large matrices.
%
%       The key idea is based on the mathematical fact that
%       the determinant of a triangular matrix equals the
%       product of its diagonal elements. Hence, the matrix's
%       log-determinant is equal to the sum of their logarithm
%       values. By keeping all computations in log-scale, the
%       problem of underflow/overflow caused by product of 
%       many numbers can be effectively circumvented.
%
%       The implementation is based on LU factorization.