% compute trace(A*S1*B*S2) where A,B are dense symmetric matrices, S1,S2 are sparse
% typically, it would be: trdsdsmat(pZUZ, Akdix, invZ, Akdjx)
function [tr]=trdsdsmat(A,S1,B,S2)

  %tic;
  %S1BS2=S1*B*S2;
  %tr = trace(A*S1BS2);
  %toc

  %tic;
  %S1BS2=sparse(S1*(full(B)*S2)); % or sparse(full(B)*S2)...?
  %tr = trace(A*S1BS2);
  %toc

  % USE ME (if not mex)!   % -->7.5s (sparse), ?? (dense)
  %tic;
  S1BS2=S1*(B*S2); % or sparse(full(B)*S2)...?
  tr = A(:)'*S1BS2(:);
  %toc

  % or use mex
  %tr=mextrdsdsmat(A,S1,full(B),S2);  % very slow... --> 15s
  %%tr=mextrdsdsmat(A,S1,B,S2);  % with full one above --> ~6s

  %S1BS2=S1*sparse((full(B)*S2)); % or sparse(full(B)*S2)...?
  %tr = A(:)'*S1BS2(:);

  % rather list the elements...?
  % zkusit tr(A,B) = sum A_ij * B_ij = svec(A)'svec(B)
  % careful about svec on symmetric matrices (sqrt(2)...)


