% This routine converts input structure PEN for PenBMI (as described 
% in PenOpt/PenBMI manual Section 5.2) into a structure accepted by 
% PenLab via BMI2 or PMI modules, for details see manual or
%    modules/BMI2/bmi2_define.m.
% Note that the convertor doesn't do any heavy checking on the input
% structure. If there were any inconsistencies in 'pen' it might be
% noticed either here or in bmi2_define().
%
% Structure 'pen' can be also obtained via YALMIP as internal input
% for PenBMI.
%
% Example:
%    load didi/pen-f4e.mat
%    didi
%    bmidata = didi2bmi2(pen,'f4e')
%    penm = bmi2_define(bmidata);
%    prob = penlab(penm);
%    prob.solve();
%    ...
%
function [bmidata]=didi2bmi2(pen, name)

  if (nargin<=1)
    name='BMI2 from convertor didi2bmi2()';
  end

  %%%%%%%%%  Convert the data %%%%%%%%%  

  bmidata = [];
  bmidata.name = name;

  % no of variables
  if (~isfield(pen,'vars') || pen.vars<=0)
    error('Input: wrong or missing vars component in pen.');
  end
  bmidata.Nx = pen.vars;
  Nx = pen.vars;

  % if present, copy starting point
  if (isfield(pen,'x0'))
    if (isempty(pen.x0) || ~isvector(pen.x0) || length(pen.x0)~=Nx)
      error('Input: starting point x0 is incompatible.');
    end
    bmidata.xinit = pen.x0;
  end

  % dense vector of the linear part of objective function
  if (isfield(pen,'fobj') && ~isempty(pen.fobj))
    bmidata.c = pen.fobj;
  end

  % if present, quadratic part of the objective function
  if (isfield(pen,'q_nzs') && ~isempty(pen.q_nzs) && pen.q_nzs>0)
    if (~isfield(pen,'q_row') || ~isfield(pen,'q_col') || ~isfield(pen,'q_val'))
      error('Input: incomplete quadratic part of the objective.');
    end
    H = sparse(pen.q_row+1, pen.q_col+1, pen.q_val,Nx,Nx,pen.Q_nzs);
    % only upper triangle, symmetrize
    bmidata.H = H + triu(H,1)';
  end

  % block of linear constraints stored in 'pen' in the form Bx<=c
  if (isfield(pen,'constr') && ~isempty(pen.constr) && pen.constr>0)
    if (~isfield(pen,'bi_dim') || ~isfield(pen,'bi_idx') || ...
      ~isfield(pen,'bi_val') || ~isfield(pen,'ci'))
      error('Input: incomplete linear constraints Bx<=c.');
    end
    if (any(pen.bi_dim<0))
      error('Input: bi_dim has negative components.');
    end

    % convert CCS -> CS
    nnzB=sum(pen.bi_dim);
    brows=[];
    for idx=1:pen.constr
      brows=[brows;idx*ones(pen.bi_dim(idx),1)];
    end
    B = sparse(brows,pen.bi_idx+1,pen.bi_val,pen.constr,Nx,nnzB);
    bmidata.B = B;
    bmidata.ubg = pen.ci;
  end

  % matrix constraints
  if (isfield(pen,'mconstr') && ~isempty(pen.mconstr) && pen.mconstr>0)
    if (~isfield(pen,'msizes') || any(pen.msizes<1))
      error('Input: msizes missing or wrong.');
    end

    % linear matrix terms - OK?
    if (isfield(pen,'ai_dim'))
      if (isempty(pen.ai_dim) || any(pen.ai_dim<0) || ...
        ~isfield(pen,'ai_idx') || ...
        ~isfield(pen,'ai_nzs') || any(pen.ai_nzs<0) || ...
        ~isfield(pen,'ai_val') || ~isfield(pen,'ai_col') || ...
        ~isfield(pen,'ai_row'))
        error('Input: incomplete linear matrix terms A_i^k.');
      end
    else
      % linear SDP terms missing? assume that all are empty
      pen.ai_dim=zeros(pen.mconstr,1);
      pen.ai_idx=zeros(1,1);
      pen.ai_nzs=zeros(1,1);
      pen.ai_val=zeros(1,1);
      pen.ai_col=zeros(1,1);
      pen.ai_row=zeros(1,1);
    end

    % bilinear matrix terms - OK?
    if (isfield(pen,'ki_dim'))
      if (isempty(pen.ki_dim) || any(pen.ki_dim<0) || ...
        ~isfield(pen,'ki_idx') || ~isfield(pen,'kj_idx') || ...
        ~isfield(pen,'ki_nzs') || any(pen.ki_nzs<0) || ...
        ~isfield(pen,'ki_val') || ~isfield(pen,'ki_col') || ...
        ~isfield(pen,'ki_row'))
        error('Input: incomplete bilinear matrix terms K_ij^k.');
      end
    else
      % bilinear SDP terms missing --> just linear SDP, no higher order matrices
      pen.ki_dim=zeros(pen.mconstr,1);
      pen.ki_idx=zeros(1,1);
      pen.kj_idx=zeros(1,1);
      pen.ki_nzs=zeros(1,1);
      pen.ki_val=zeros(1,1);
      pen.ki_col=zeros(1,1);
      pen.ki_row=zeros(1,1);
    end

    bmidata.Na = pen.mconstr;
    bmidata.A = cell(pen.mconstr,1);

    % copy one matrix constraint at a time
    midxAstart=1;          % indices to *_nzs() and *_idx() arrays
    midxKstart=1;
    idxA=1;                % indices to *_val, *_col, *_row arrays
    idxK=1;

    for k=1:pen.mconstr
      midxAend=midxAstart+pen.ai_dim(k)-1;
      midxKend=midxKstart+pen.ki_dim(k)-1;
      nnzAsum=sum(pen.ai_nzs(midxAstart:midxAend));
      nnzKsum=sum(pen.ki_nzs(midxKstart:midxKend));

      bmidata.A{k} = copy_mconstr(pen.msizes(k),pen.ai_dim(k),pen.ki_dim(k),...
        pen.ai_idx(midxAstart:midxAend), ...
        pen.ki_idx(midxKstart:midxKend), pen.kj_idx(midxKstart:midxKend), ...
        pen.ai_nzs(midxAstart:midxAend), ...
        pen.ai_val(idxA:end), pen.ai_row(idxA:end), pen.ai_col(idxA:end),...
        pen.ki_nzs(midxKstart:midxKend), ...
        pen.ki_val(idxK:end), pen.ki_row(idxK:end), pen.ki_col(idxK:end));


      % move indices in *_nzs arrays
      midxAstart=midxAend+1;
      midxKstart=midxKend+1;
      % move indices in *_val arrays
      idxA = idxA+nnzAsum;
      idxK = idxK+nnzKsum;
    end
  end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% create one matrix constraints from PenBMI data
% in the format required in BMI2/PMI
%
% Input:
%   dim - dimension of the matrix constraint
%   nA, nK - number of matrices in linear and bilinear terms
%   A_idx - dim nA, indices of the linear matrices A_i
%   K_idx, K_jdx - dim nK, i and j indices of bilinear matrices K_ij
%   A_nnz - dim nA, number of nonzeros in each given matrix A_i
%   A_val,col,row - dim sum(A_nnz), matries A_i next to each other
%     stored in CS upper triangle format, 0-based indices row/col
%   K_nnz - dim nK, number of nonzeros in each given K_ij
%   K_val,col,row - dim sum(K_nnz), K_ij matrices, same format as A_i
%
function [Ak] = copy_mconstr(dim,nA,nK,A_idx,K_idx,K_jdx,...
  A_nnz,A_val,A_row,A_col,K_nnz,K_val,K_row,K_col)

  if (nA==0 && nK==0)
    error('Input: neither linear nor bilinear matrices in this constraint.');
  end

  if (nK==0)
    maxOrder = 1;
    Ak.midx = zeros(maxOrder,nA);
    Ak.midx(1,:) = A_idx(1:nA);
  else
    maxOrder = 2;
    Ak.midx = zeros(maxOrder,nA+nK);
    Ak.midx(1,1:nA) = A_idx(1:nA);   % midx(2,1:nA) = zeroes()
    Ak.midx(1,nA+1:end) = K_idx(1:nK);
    Ak.midx(2,nA+1:end) = K_jdx(1:nK);
  end
  
  Ak.Q = cell(nA+nK,1);
  % copy linear terms
  idx = 1;          % index to _val/_row/_col arrays
  for midx=1:nA
    idxend = idx+A_nnz(midx)-1;
    Qi = sparse(A_row(idx:idxend)+1, A_col(idx:idxend)+1, A_val(idx:idxend),...
      dim, dim, A_nnz(midx));
    % symmetrize, was just upper triangle
    Qi = Qi + triu(Qi,1)';
    % swap sign, we want Ak >= 0 (pos. def.) and it was neg. def.
    Ak.Q{midx} = -Qi;

    idx = idxend+1;
  end

  % copy bilinear terms
  idx = 1;          % index to _val/_row/_col arrays
  for midx=1:nK
    idxend = idx+K_nnz(midx)-1;
    Qi = sparse(K_row(idx:idxend)+1, K_col(idx:idxend)+1, K_val(idx:idxend),...
      dim, dim, K_nnz(midx));
    % symmetrize, was just upper triangle
    Qi = Qi + triu(Qi,1)';
    % swap sign, we want Ak >= 0 (pos. def.) and it was neg. def.
    Ak.Q{nA+midx} = -Qi;

    idx = idxend+1;
  end

end

