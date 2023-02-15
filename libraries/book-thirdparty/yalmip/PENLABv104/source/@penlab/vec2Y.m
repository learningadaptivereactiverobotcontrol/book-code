% Map 'vectorized' matrix decision variables back to cell array of matrices Y
% by usage of obj.vec2Ymap, see createY1map() for details 
%   vec  ... vector of length obj.NYnnz
%   Y  ... cell array obj.NY x 1 with filled in 'vec' to the proper nnz places
%       Y is returned as symmetric
function  Y=vec2Y(obj, vec)
  
  % perhaps skip this?
  if (length(vec) ~= obj.NYnnz)
    error('map vec->Y, wrong length of vec');
  end

  if (obj.NY>0)
    Y=cell(obj.NY,1);

    for k=1:obj.NY
      % create Y{k} as appropriate
      mapper=obj.vec2Ymap{k};
      if (mapper.dense)
        %Y{k}=zeros(mapper.dim);
        %Y{k}(:)=mapper.xmap(vec);
        Y{k}=reshape(vec(mapper.xmap),mapper.dim, mapper.dim);
      else
        % sparse
        Y{k}=sparse(mapper.irow, mapper.icol, vec(mapper.xmap), mapper.dim, mapper.dim);
        %sparse(i,j,s,m,n)
      end
    end

  else
    Y=[];
  end
end
