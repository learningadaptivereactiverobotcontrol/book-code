function delta = tKLFunc(p,q)
%% total Kullback Leibler divergence (tKL) delta_f(p,q) between two
%% multivariate normal probabiltiy density functions.
% f is the convex and differentiable generator function
% f is the negative entropy;

%% Input: 
%     p: Gaussian distribution with mean 0 and variance sigma;
%     q: Gaussian distribution with mean 0 and variance sigma;
% Output: 
%     delta: the tKL between p and q.
%% Signature 
%   Author: Meizhu Liu
%   E-Mail: mliu@cise.ufl.edu
%   Date  : December 28 2010
%% References:
%   Baba C. Vemuri, Meizhu Liu, Shun-Ichi Amari and Frank Nielsen, 
%   Total Bregman Divergence and its Applications to DTI Analysis, 
%   IEEE Transactions on Medical Imaging (TMI'10), 2010.
% 
%   Meizhu Liu, Baba C. Vemuri, Shun-Ichi Amari and Frank Nielsen, 
%   Total Bregman Divergence and its Applications to Shape Retrieval, 
%   IEEE Conference on Computer Vision and Pattern Recognition (CVPR’10),
%   2010.

%% Example Usage: 
%     p.sigma = [1 0;0 1];
%     q.sigma = [2 0;0 2];;
%     delta = tKLFunc(p,q);

%% The main part
P = p.sigma;
Q = q.sigma;
invP = inv(P);
invQ = inv(Q);
d = size(Q,1); %the dimension of the dataset following this distribution
c = 0.75*d+d^2*log(2*pi)/2+(d*log(2*pi))^2/4;
delta = (log(det(invP*Q))+trace(invQ*P)-d);
tmp = c+log(det(Q))^2/4-d*(1+log(2*pi))/2*log(det(Q));
delta = delta/2/sqrt(tmp);
end
