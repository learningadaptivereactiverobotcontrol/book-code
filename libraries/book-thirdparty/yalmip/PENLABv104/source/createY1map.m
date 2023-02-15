% Check & create one structure representing one matrix variable Y 
% The structure will be one element of array vec2Ymap used in vec2Y().
%   Y ... "pattern" matrix given by user (all nnz in one triangle -> variables)
%   offset ... gives the last number of the variable (matrix element considered
%     as variable) so far, so typically obj.NYnnz and 0 at the beggining;
%     matrix element/variables will start with number (offset+1)
%     and the returned offset is the last one
%   mapper ... structure mapping vectorized variable back to Y in a fast way
%     .dim   ... dimxdim dimension of the matrix variable
%     .nelem ... no of variables (=no of elements in lower triangle)
%     .dense ... generate as dense (true) or sparse (false)
%     .xmap  ... mapping the (whole) vector of (Yall) elements) -> nnz of one Y
%        the first one (xmap(1)) is always the same as offset+1, i.e., first
%        variable (counting 1...NYnnz) in this matrix;
%        thus this matrix is created by variables xmap(1)..xmap(1)+nelem-1
%     .irow, .icol  ... to generate sparse matrix with xmap (if ~dense),
%        first nelem refer to the lower triangle, the rest to the upper;
%        if dense, the length is nelem and they map to the lower triangle,
%        it could be computed because it'll be 1,1; 2,1; ...dim,1; 2,2; 3,2;...
% Note, to see exactly where the given variable maps (= its derivative):
%   variable with index idx=1..nelem maps to irow(idx),icol(idx) + symmetric
%   if nondiagonal, absolute number of such a variable (in xall()) is
%   idx=Nx+xmap(1), ... ,Nx+xmap(1)+nelem-1
function [mapper, offset]=createY1map(Y,offset)
  mapper=[];
  
  % take empty as OK? Yes, such a matrix will be filtered-out from constraints
  if (isempty(Y))
    mapper.dim=0;
    mapper.nelem=0;
    mapper.dense=true;
    mapper.xmap=[];
    mapper.irow=[];
    mapper.icol=[];
  else
    % does Y look OK?
    [dim, dim2] = size(Y);
    if (dim~=dim2)
      error('Y is not a square??');
      return;
    end

    if (~issparse(Y) && nnz(Y)==dim*dim)
      % Y is really dense and the full lower triangle will create decision vars
      nonnz=dim*(dim+1)/2;
      % do it better???
      [row,col]=find(tril(Y));  % should be 1,1; 2,1; 3,1; 2,2; 3,2; ...
      mat=full(sparse(row,col,[1:nonnz]));
      mat=mat+tril(mat,-1)';

      mapper.dim=dim;
      mapper.nelem=nonnz;
      mapper.dense=true;
      mapper.xmap=offset + mat(:);  % should be same as reshape(mat,dim*dim,1)
      %mapper.irow=[];
      %mapper.icol=[];
      mapper.irow=row;
      mapper.icol=col;
      offset=offset+nonnz;

    else
      % work with it as sparse
      Y=spones(Y);   % to avoid random nullification in Y+Y'
      Y=tril(Y+Y');  % or don't symmetrize? or check?
      [row, col] = find(Y);
      inner=find(row~=col);
      nonnz=length(row);

      mapper.dim=dim;
      mapper.nelem=nonnz;
      mapper.dense=false;
      mapper.irow=[row; col(inner)];
      mapper.icol=[col; row(inner)];
      mapper.xmap= offset + [[1:nonnz]'; inner];
      offset = offset + nonnz;
    end

  end

