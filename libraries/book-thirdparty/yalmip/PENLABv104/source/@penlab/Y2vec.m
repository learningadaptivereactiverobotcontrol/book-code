% Copy matrix data (with respect to the given pattern) to xall,
% only data from the lower triangle and on the pattern will be used (the rest
% is ignored). Quite slow but used only during initialization (e.g., Yinit).
%
% Be careful that here Y can be just part of full length obj.Y,
% imagine obj.Yinit{2}=...; then Y{1}=[], Y{2}=... and there is no Y{k>=3} !
% Ydefault specifies what should be the default value if Y{k} is not present
% on input or it is an empty matrix, possibilities:
%   -/+Inf/0 ... vector of all -/+Inf/0
%   anything else (or not present) ... the current point
% Y{k} is empty or not present at all --> default value
%      is of the same size as internal Y --> only lower triangle elements
%      matching the pattern the internal Y will be considered
%      is a scalar --> this value will be distributed to all Y in the pattern
%      (eg. acts as if you put Y{k}=scalar*ones(size(Ymap{k}))
function vec = Y2vec(obj, Y, Ydefault)

  % use a point to fill in the gabs if there are in Y
  % if Y is a full size array, it won't influence it anyway
  % important only for Y being partially assigned
  if (nargin<=2)
    % current point
    vec=obj.xall(obj.Nx+1:end);
  else
    if (Ydefault==-Inf)
      vec=-Inf(obj.NYnnz,1);
    elseif (Ydefault==Inf)
      vec=Inf(obj.NYnnz,1);
    elseif (Ydefault==0)
      vec=zeros(obj.NYnnz,1);
    else
      vec=obj.xall(obj.Nx+1:end);
    end
  end

  if (length(Y)>obj.NY)
    error('Incompatible Y on input, cannot transfer data');
  end

  for k=1:length(Y)
    if (~isempty(Y{k}))
      mapper=obj.vec2Ymap{k};

      [dim, dim2] = size(Y{k});
      if (dim==1 && dim2==1)
        % scalar --> apply this bound on all elements
        vec(mapper.xmap)=Y{k};

      elseif (dim==dim2 && dim==mapper.dim)
        % matching dimension, get the appropriate elements

        if (mapper.dense)
          % mapper.xmap includes double assignments (from both triangles) 
          % to be sure which half of the user's data gets used, symmetrize
          % and use lower only

          Ytmp=tril(Y{k}) + tril(Y{k},-1)';
          vec(mapper.xmap)=reshape(full(Ytmp),dim*dim,1);
        else
          len=length(mapper.xmap);
          % have to do it element by element because MATRIX(irow,icol) will
          % be a submatrix not an array of arguments. Find is also not suitable
          % because it won't register zeros (which are on the pattern of variables)
          for idx=1:len
            ridx=mapper.irow(idx);
            cidx=mapper.icol(idx);
            if (ridx>=cidx)
              % lower triangle only
              vec(mapper.xmap(idx))=Y{k}(ridx,cidx);
            end
          end
        end
      else
        % not matching dimension and not a scalar (dim~=dim2 || dim~=mapper.dim)
        error('Y{k} has different size.');
      end
    end
  end



