% As eval_alx(), evaluate gradient of the Augmented Lagrangian and Jacobian
% of the equality constraints. The results are stored in obj.ALdx, obj.eqdx
% and tickers updated.
function [status] = eval_aldx(obj)

  % status TODO
  status = 0;

  if (obj.ALdxtck < obj.ticker)
    starttime = cputime;

    % create local copies of obj.x,obj.Y to avoid checking repetitively if they
    % are 'up-to-date' with obj.xall, they get used many times in calls to
    % user's functions
    x=obj.x;
    Y=obj.Y;
    % same for obj.userdata, it might be quite expensive to get&store it in
    % the object
    userdata=obj.userdata;

    % TODO reuse it + 'pointchanged' flag
    [fx, userdata] = obj.objfun(x, Y, userdata);
    [fdx, userdata] = obj.objgrad(x, Y, userdata);

    % function inequal & equal
    if (obj.NgNLN + obj.NgLIN>0)
      [gx, userdata] = obj.confun(x, Y, userdata);
      ineqx = obj.ineqshift + obj.ineqmlt .* gx(obj.ineqmap);
      obj.eqx = obj.eqshift + gx(obj.eqmap);

      [gdx, userdata] = obj.congrad(x, Y, userdata);
      % because of stupid spdiags() which doesn't accept ([],0,0,0)
      if (obj.Nineq>0)
        ineqdx = gdx(:,obj.ineqmap) * spdiags(obj.ineqmlt,0,obj.Nineq,obj.Nineq);
      else
        ineqdx = [];
      end
      % TODO update ticker eqdx
      obj.eqdx = gdx(:,obj.eqmap);
    end

    % box constraints
    if (obj.Nxbox>0)
      xboxx = obj.xboxshift + obj.xboxmlt .* obj.xall(obj.xboxmap);
      xboxdx = sparse(obj.xboxmap,[1:obj.Nxbox],obj.xboxmlt,obj.Nx,obj.Nxbox);
    end

    ALdx=fdx;
    
    % box constraints
    ind=obj.xboxindbar;
    if (~isempty(ind))
      ALdx = ALdx + xboxdx(:,ind)*(obj.uxbox(ind).*obj.pxbox(ind).*obj.phibar_D(xboxx(ind)));
    end

    ind=obj.xboxindphi;
    if (~isempty(ind))
      ALdx = ALdx + xboxdx(:,ind)*(obj.uxbox(ind).*obj.phi2_D(xboxx(ind)./obj.pxbox(ind)));
    end

    % function inequalitites
    %ind=obj.ineqindbar;
    %... TODO

    ind=obj.ineqindphi;
    if (~isempty(ind))
      ALdx = ALdx + ineqdx(:,ind)*(obj.uineq(ind).*obj.phi2_D(ineqx(ind)./obj.pineq(ind)));
    end

    % (function) equalities
    if (obj.Neq>0)
      ALdx = ALdx + obj.eqdx*obj.ueq;
    end

    % matrix variable - log barrier (strict feasibility)
    if ~isempty(obj.Yboxindbar)
    for k=obj.Yboxindbar
      % convert the matrix box constraint to the form:   +/-Y +/-bound >=0
      pkx=obj.PYbox(k);
      Ykx = Y{obj.Yboxmap(k)};
      Akx=-obj.Yboxshift(k)*speye(size(Ykx)) - obj.Yboxmlt(k)*Ykx;

      % can assume that ALx was computed before ALdx --> Akx must be pos. def.
      chol(Akx);
      invAkx = full(inv(Akx));

      % compute partial derivatives of the penalty term:
      %    d/dxi... = -p*trace( (A^-1) * (d/dxi A) )

      % there should be just one dependant with obvious value (+-1) in each
      % triangle of every Y{k} derivative
      mlt=-obj.Yboxmlt(k);  % +/-1
      mapper=obj.vec2Ymap{obj.Yboxmap(k)};
      dim=mapper.dim;
      offset=obj.Nx + mapper.xmap(1) - 1;
      irow=mapper.irow;
      icol=mapper.icol;
      for idx=1:mapper.nelem
        if (irow(idx)==icol(idx))
          % diagonal element
          Akdx = sparse(irow(idx),icol(idx),mlt,dim,dim);
        else
          % nondiag element --> add two
          Akdx = sparse([irow(idx),icol(idx)],[icol(idx),irow(idx)],[mlt,mlt],dim,dim);
        end
        % but this can be done directly...!
        ALdx(offset+idx) = ALdx(offset+idx) - pkx*trace(invAkx*Akdx);
      end

    end
    end

    % matrix variable - pen/bar
    if ~isempty(obj.Yboxindphi)
    for k=obj.Yboxindphi
      pkx=obj.PYbox(k);            %2*p(sdpdata.Ng+k);
      Ykx = obj.Y{obj.Yboxmap(k)};
      Akx=obj.Yboxshift(k)*speye(size(Ykx)) + obj.Yboxmlt(k)*Ykx;
      umatk=obj.UYbox{k};

      %Z=(pkx*speye(size(Akx))-Akx);
      Z=(pkx*speye(size(Akx))-Akx);
      %invZ=inv(Z);
      invZ=full(inv(Z));
      pZUZ=pkx^2*invZ*umatk*invZ;

      % there should be just one dependant with obvious value (+-1) in each
      % triangle of every Y{k} derivative
      mlt=obj.Yboxmlt(k);  % +/-1
      mapper=obj.vec2Ymap{obj.Yboxmap(k)};
      dim=mapper.dim;
      offset=obj.Nx + mapper.xmap(1) - 1;
      irow=mapper.irow;
      icol=mapper.icol;
      for idx=1:mapper.nelem
        if (irow(idx)==icol(idx))
          % diagonal element
          Akdx = sparse(irow(idx),icol(idx),mlt,dim,dim);
        else
          % nondiag element --> add two
          Akdx = sparse([irow(idx),icol(idx)],[icol(idx),irow(idx)],[mlt,mlt],dim,dim);
        end
        % but this can be done directly...!
        ALdx(offset+idx) = ALdx(offset+idx) + trace(pZUZ*Akdx);
      end
    end
    end

    % matrix constraints - pen/bar
    if ~isempty(obj.Aindphi)
    for k=obj.Aindphi
      pkx=obj.PA(k);  % I used to use 2*         !!!!!!!!
      % TODO need to map the matrix first! - is it correct???
      kuser=obj.Amap(k);
      [Akuserx, userdata] = obj.mconfun(x, Y, kuser, userdata);
      Akx = obj.Ashift(k)*speye(size(Akuserx)) + obj.Amlt(k) .* Akuserx;
      umatk=obj.UA{k};

      %Z=(pkx*speye(size(Akx))-Akx);
      Z=(pkx*speye(size(Akx))-Akx);
      %invZ=inv(Z);
      invZ=full(inv(Z));
      pZUZ=pkx^2*invZ*umatk*invZ;
      
      % MAPPING & transformation !!!
      for i=obj.Adep{kuser}
        [Akdx, userdata] = obj.mcongrad(x,Y,kuser,i,userdata);
        Akdx = obj.Amlt(k)*Akdx;
        %ALdx(i) = ALdx(i) + trace(pZUZ*Akdx);
        ALdx(i) = ALdx(i) + pZUZ(:)'*Akdx(:);
      end
    end
    end

    % store user's data back in the object if it got changed
    obj.userdata=userdata;

    % update ticker
    obj.ALdx = ALdx;
    obj.ALdxtck = obj.ticker;

    % update stats
    obj.stats_ncall_aldx = obj.stats_ncall_aldx + 1;
    obj.stats_time_aldx = obj.stats_time_aldx + cputime - starttime;

  end


