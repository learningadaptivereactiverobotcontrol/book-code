% Lagrangian Multipliers (for Matrix constraints) update
function [] = lmltm_update(obj)

  % obviously only pen/bar 
  % I have in the object: UYbox, UYboxnew, UA, UAnew - cell array with indices
  % matching Yboxindphi & Aindphi

  mu2 = obj.allopts.mmlt_update;

  % compute new candidates in *new; this will be done in eval*() as a by-product

  % matrix variable - pen/bar
  for k=obj.Yboxindphi
    pkx=obj.PYbox(k);            %2*p(sdpdata.Ng+k);
    %Akx=evalax(x,k,sdpdata);
    % mapping xall first !!! TODO take obj.Y{...}
    Ykx = obj.Y{obj.Yboxmap(k)};
    Akx=obj.Yboxshift(k)*speye(size(Ykx)) + obj.Yboxmlt(k)*Ykx;
    umatk=obj.UYbox{k};

    %Z=(pkx*speye(size(Akx))-Akx);
    Z=(pkx*speye(size(Akx))-Akx);
    invZ=inv(Z);
    pZUZ=pkx^2*invZ*umatk*invZ;
    obj.UYboxnew{k}=pZUZ;
  end

  % matrix constraints - pen/bar
  for k=obj.Aindphi
    pkx=obj.PA(k);  % I used to use 2*         !!!!!!!!
    % TODO need to map the matrix first! - is it correct???
    kuser=obj.Amap(k);
    [Akuserx, obj.userdata] = obj.mconfun(obj.x, obj.Y, kuser, obj.userdata);
    Akx = obj.Ashift(k)*speye(size(Akuserx)) + obj.Amlt(k) .* Akuserx;
    umatk=obj.UA{k};

    %Z=(pkx*speye(size(Akx))-Akx);
    Z=(pkx*speye(size(Akx))-Akx);
    invZ=inv(Z);
    pZUZ=pkx^2*invZ*umatk*invZ;
    obj.UAnew{k}=pZUZ;
  end

  % update as in update_multipliers() @ pbmfnc/Pennon1.0
  % [in SDP/BMI there is still active one more option]
  %  rStabilize = 1.0E-16*sqrt((double)(Umat->mat[i]->m)*diagnorm(Umat->mat[i], nSparse)); 
  %  m_stabilize(Umat->mat[i],  rStabilize, nSparse);  
  %    rLambda = 1-mu2;
  %    m_ConvComb(rLambda, Umattmp->mat[i], Umat->mat[i], Umat->mat[i], nSparse);
  %  rStabilize = 1.0E-16*sqrt((double)(Umat->mat[i]->m)*diagnorm(Umat->mat[i], nSparse)); 
  %  m_stabilize(Umat->mat[i],  rStabilize, nSparse);
  % Note that in the code Umat & Umattmp are swapped (Umat is the new one in
  % this part)

  % matrix variable - pen/bar
  for k=obj.Yboxindphi
    umatk = obj.UYboxnew{k};
    umatk = m_stabilize(umatk);
    %lambda=1-mu2;
    %umatk = (1-lambda)*old + lambda*new;
    umatk = mu2*obj.UYbox{k} + (1-mu2)*umatk;
    obj.UYbox{k} = m_stabilize(umatk);
    % & push it back but keep the 'old ones'??
    % are they used behind update_multipliers??
  end

  % matrix constraints - pen/bar
  for k=obj.Aindphi
    umatk = obj.UAnew{k};
    umatk = m_stabilize(umatk);
    %lambda=1-mu2;
    %umatk = (1-lambda)*old + lambda*new;
    umatk = mu2*obj.UA{k} + (1-mu2)*umatk;
    obj.UA{k} = m_stabilize(umatk);
    % & push it back but keep the 'old ones'??
    % are they used behind update_multipliers??
  end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% stabilize M
% only on diag elements: m_ii = |m_ii| + r
% where r is given by the norm of the diag
% how to access the diagonal of a dense matrix?
%   M(1:n+1:end) = newdiag
%   iii=1:n+1:n^2; M(iii)=...   [slightly slower]
%   or normal for loop
function [M] = m_stabilize(M)
  [n m] = size(M);
  Mdiag = diag(M);
  rStabilize = 1.0e-16*realsqrt(n*norm(Mdiag,2)); 
  M(1:n+1:end) = abs(Mdiag) + rStabilize;
end


