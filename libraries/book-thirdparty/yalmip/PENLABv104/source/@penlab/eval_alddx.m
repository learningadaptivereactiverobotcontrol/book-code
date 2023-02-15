% As eval_alx(), evaluate hessian of the Augmented Lagrangian and Jacobian
% of the equality constraints. The results are stored in obj.ALddx, obj.eqdx
% and tickers updated.
function [status] = eval_alddx(obj)

  % status TODO
  status = 0;

  % TODO remove!
  if (true || obj.ALddxtck < obj.ticker)
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

    % prepare Lmlt for all function 2nd derivatives fddx + Lmlt*gddx
    % (in user's indices, including both equal & inequal)
    % AMPL wants to have the full length ... but in fact only obj.NgNLN
    % should be enough.  ==> try with all and then reduce it later TODO
    % TODO vectorize
    % TODO this is right only if all phiind! (which will be probably true,
    % as no NLN constr should go to the barrier ... why should they?)
    if (obj.Nineq>0)
      ind=1:obj.Nineq;
      Lmlt_long=obj.ineqmlt(ind).*obj.uineq(ind).*obj.phi2_D(ineqx(ind)./obj.pineq(ind));
      % gather these Lmlt belonging to the same "body" of the constraint
      % (sparse sums element which belong to the same index)
      Lmlt = full(sparse(obj.ineqmap(ind),1,Lmlt_long,obj.NgNLN+obj.NgLIN,1));
    else
      Lmlt = zeros(obj.NgNLN+obj.NgLIN,1);
    end
%    Lmlt = zeros(obj.NgNLN+obj.NgLIN,1);
%    for k=1:obj.Nineq
%      kuser=obj.ineqmap(k);
%      Lmlt(kuser) = Lmlt(kuser) + obj.ineqmlt(k)*obj.uineq(k)*obj.phi2_D(ineqx(k)/obj.pineq(k));
%    end
    for k=1:obj.Neq
      kuser=obj.eqmap(k);
      Lmlt(kuser)=obj.ueq(k);
    end

    % get Lagrangian directly or compose it
    if (~isempty(obj.lagrhess))
      [ALddx, userdata] = obj.lagrhess(x,Y,Lmlt,userdata);
      if (isempty(ALddx))
        ALddx=sparse(obj.Nx+obj.NYnnz,obj.Nx+obj.NYnnz);
      end
    else
      % compose Lagrangian
      [ALddx, userdata] = obj.objhess(x,Y,userdata);
      if (isempty(ALddx))
        ALddx=sparse(obj.Nx+obj.NYnnz,obj.Nx+obj.NYnnz);
      end
      for kuser=1:obj.NgNLN
        [gddx, userdata] = obj.conhess(x,Y,kuser,userdata);
        ALddx = ALddx + Lmlt(kuser)*gddx;
      end
    end

    %%% Add dyadic product of function inequalitites (and box)

    % box constraints
    if (obj.Nxbox>0)
      xboxx = obj.xboxshift + obj.xboxmlt .* obj.xall(obj.xboxmap);
      % this adds only diagonal scaling as grad = (0,...,1,...0) or with -1
      % so it will be either:
      %   + u(i)/p(i)*phi2_D2(g(i)/p(i))   for phi/bar
      % or 
      %   + u(i)*p(i)*phibar_D2(g(i))      for barrier
      % or nothing if not bounded, don't forget on both inequalitites
      diagxbox=zeros(obj.Nx+obj.NYnnz,1);
      % TODO vectorize
      ind=obj.xboxindbar;
      if ~isempty(ind)
      for k=ind
        kuser=obj.xboxmap(k);
        diagxbox(kuser) = diagxbox(kuser) + obj.uxbox(k)*obj.pxbox(k)*obj.phibar_D2(xboxx(k));
      end
      end

      ind=obj.xboxindphi;
      if ~isempty(ind)
      for k=ind
        kuser=obj.xboxmap(k);
        diagxbox(kuser) = diagxbox(kuser) + obj.uxbox(k)/obj.pxbox(k)*obj.phi2_D2(xboxx(k)/obj.pxbox(k));
      end
      end
      ALddx = ALddx + spdiags(diagxbox,0,obj.Nx+obj.NYnnz,obj.Nx+obj.NYnnz);
    end

    % function inequalitites
    %ind=obj.ineqindbar;
    %... TODO

    ind=obj.ineqindphi;
    if (~isempty(ind))
      coef=obj.uineq(ind)./obj.pineq(ind).*obj.phi2_D2(ineqx(ind)./obj.pineq(ind));
      ALddx = ALddx + ineqdx(:,ind) * spdiags(coef,0,length(ind),length(ind)) * ineqdx(:,ind)';

    end
%    for k=ind
      %k
      %nnz(ineqdx(:,k))
%      ALddx = ALddx + ((obj.uineq(k)/obj.pineq(k)*obj.phi2_D2(ineqx(k)/obj.pineq(k)))*ineqdx(:,k))*ineqdx(:,k)';
      %disp('k done')
      %nnz(ALddx)
%    end

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

      % compute elements of 2nd derivative of the penalty term:
      %   d2/dxi dxj ... = p*trace( (A^-1) * (d/dxi A) * (A^-1) * (d/dxj A) )

      mlt=-obj.Yboxmlt(k);  % +/-1
      mapper=obj.vec2Ymap{obj.Yboxmap(k)};
      dim=mapper.dim;
      lAdep=mapper.nelem;
      offset=obj.Nx + mapper.xmap(1) - 1;
      irow=mapper.irow;
      icol=mapper.icol;

      Akdixall=cell(lAdep,1);
      % store everything in a dense matrix (kernel) and copy it to the Hessian
      % once when finished
      Akddx=zeros(lAdep,lAdep);
      for ii=1:lAdep
        if (irow(ii)==icol(ii))
          % diagonal element
          Akdix = sparse(irow(ii),icol(ii),mlt,dim,dim);
        else
          % nondiag element --> add two
          Akdix = sparse([irow(ii),icol(ii)],[icol(ii),irow(ii)],[mlt,mlt],dim,dim);
        end

        Akdixall{ii}=Akdix;

        % compute all elements under the diagonal
        for jj=1:ii-1
          hij=0.5*pkx*mextrdsdsmat(invAkx, Akdix, invAkx, Akdixall{jj});
          Akddx(ii,jj)=hij;
          Akddx(jj,ii)=hij;
        end

        % compute diagonal element jj=ii
        hij=0.5*pkx*mextrdsdsmat(invAkx, Akdix, invAkx, Akdix);
        Akddx(ii,ii)=hij;
        
      end
      Adep=offset+[1:lAdep];
      % copy the dense kernel back to ALddx
      % ALddx=ALddx + Akddx(obj.Adep{kuser},obj.Adep{kuser}); ... need other way
      Akddx_expand=sparse(obj.Nx+obj.NYnnz,obj.Nx+obj.NYnnz);
      %Akddx_expand(obj.Adep{kuser},obj.Adep{kuser})=Akddx;
      Akddx_expand(Adep,Adep)=Akddx;
      ALddx=ALddx + Akddx_expand;

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

      mlt=obj.Yboxmlt(k);  % +/-1
      mapper=obj.vec2Ymap{obj.Yboxmap(k)};
      dim=mapper.dim;
      lAdep=mapper.nelem;
      offset=obj.Nx + mapper.xmap(1) - 1;
      irow=mapper.irow;
      icol=mapper.icol;

      Akdixall=cell(lAdep,1);
      % store everything in a dense matrix (kernel) and copy it to the Hessian
      % once when finished
      Akddx=zeros(lAdep,lAdep);
      for ii=1:lAdep
        if (irow(ii)==icol(ii))
          % diagonal element
          Akdix = sparse(irow(ii),icol(ii),mlt,dim,dim);
        else
          % nondiag element --> add two
          Akdix = sparse([irow(ii),icol(ii)],[icol(ii),irow(ii)],[mlt,mlt],dim,dim);
        end

        Akdixall{ii}=Akdix;

        % compute all elements under the diagonal
        for jj=1:ii-1
          hij=mextrdsdsmat(pZUZ, Akdix, invZ, Akdixall{jj});
          Akddx(ii,jj)=hij;
          Akddx(jj,ii)=hij;
        end

        % compute diagonal element jj=ii
        hij=mextrdsdsmat(pZUZ, Akdix, invZ, Akdix);
        Akddx(ii,ii)=hij;
        
      end
      Adep=offset+[1:lAdep];
      % copy the dense kernel back to ALddx
      % ALddx=ALddx + Akddx(obj.Adep{kuser},obj.Adep{kuser}); ... need other way
      Akddx_expand=sparse(obj.Nx+obj.NYnnz,obj.Nx+obj.NYnnz);
      %Akddx_expand(obj.Adep{kuser},obj.Adep{kuser})=Akddx;
      Akddx_expand(Adep,Adep)=Akddx;
      ALddx=ALddx + Akddx_expand;

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

      Z=(pkx*speye(size(Akx))-Akx);
      %invZ=inv(Z);
      invZ=full(inv(Z));
      %invZ=inv(full(Z));
      pZUZ=pkx^2*invZ*umatk*invZ;
      
      % MAPPING & transformation !!!
      Adep=obj.Adep{kuser};
      lAdep=length(Adep);
      Akdixall=cell(lAdep,1);
      % store everything in a dense matrix (kernel) and copy it to the Hessian
      % once when finished
      %Akddx=zeros(lAdep,lAdep);
      Akddx_nnz=lAdep*(lAdep+1)/2;
      Akddx_val=zeros(Akddx_nnz,1);
      Akddx_row=zeros(Akddx_nnz,1);
      Akddx_col=zeros(Akddx_nnz,1);
      idx = 0;
      for ii=1:lAdep
        i=Adep(ii);
        [Akdix, userdata] = obj.mcongrad(x,Y,kuser,i,userdata);
        if (obj.Amlt(k)<0)    % TODO correct????
          % must be -1, otherwise it would be 1
          Akdix = -Akdix;
        end
        %Akdix = obj.Amlt(k)*Akdix;
        Akdixall{ii}=Akdix;
        Akddx_row(idx+1:idx+ii)=i;
        Akddx_col(idx+1:idx+ii)=Adep(1:ii);
        % compute all elements under the diagonal
%        for jj=1:ii-1
          %---Akdjx=Akdixall{jj};

          %AZA=Akdix*invZ*Akdjx;
          %hij = trace(pZUZ*AZA);
          %%hij=trdsdsmat(pZUZ, Akdix, invZ, Akdjx);
          %---hij=mextrdsdsmat(pZUZ, Akdix, invZ, Akdjx);
%          hij=mextrdsdsmat(pZUZ, Akdix, invZ, Akdixall{jj});
          %Akddx(ii,jj)=hij;
          %Akddx(jj,ii)=hij;
%          Akddx_val(idx+jj)=hij;
%        end

        % compute diagonal element jj=ii
%        hij=mextrdsdsmat(pZUZ, Akdix, invZ, Akdix);
        %Akddx(ii,ii)=hij;
%        Akddx_val(idx+ii)=hij;

        vec = mextrcolumn(pZUZ,Akdix,invZ,Akdixall(1:ii));
        Akddx_val(idx+1:idx+ii)=vec;
        
        idx = idx+ii;
      end

      % add 2nd order matrix constraints derivatives
      % either by lagrangian or element by element
      Akddx_lagr = [];
      if (kuser<=obj.NANLN)
        if (~isempty(obj.mconlagrhess))
          [Akddx_lagr, userdata] = obj.mconlagrhess(x,Y,kuser,pZUZ,userdata);
          if (obj.Amlt(k)~=1)
            Akddx_lagr = obj.Amlt(k)*Akddx_lagr;
          end
        else
          % lagrangian is not present, do it element by element
          %Akddx_val2=zeros(Akddx_nnz,1);
          idx = 0;
          for ii=1:lAdep
            i=Adep(ii);
            for jj=1:ii-1
              %[Akddijx, userdata] = obj.mconhess(x,Y,kuser,i,Adep(jj),userdata);
              [Akddijx, userdata] = obj.mconhess(x,Y,kuser,Adep(jj),i,userdata);
              if (~isempty(Akddijx))
                % TODO better
                % don't forget a possible transformation with Akddijx,
                % can be applied on the trace (instead of on Akddijx
                % directly)
                hij=obj.Amlt(k)*trace(Akddijx*pZUZ);
                %Akddx(ii,jj)=Akddx(ii,jj)+hij;
                %Akddx(jj,ii)=Akddx(jj,ii)+hij;
                Akddx_val(idx+jj)=Akddx_val(idx+jj)+hij;
                %Akddx_val2(idx+jj)=Akddx_val2(idx+jj)+hij;
              end
            end
            [Akddijx, userdata] = obj.mconhess(x,Y,kuser,i,i,userdata);
            if (~isempty(Akddijx))
              % TODO better
              hij=obj.Amlt(k)*trace(Akddijx*pZUZ);
              %Akddx(ii,ii)=Akddx(ii,ii)+hij;
              Akddx_val(idx+ii)=Akddx_val(idx+ii)+hij;
              %Akddx_val2(idx+ii)=Akddx_val2(idx+ii)+hij;
            end
            idx = idx+ii;
          end
        end
      end
      % only lower triangle -> copy to full
      %Akddx = tril(Akddx) + tril(Akddx,-1)';
      % copy the dense kernel back to ALddx
      % ALddx=ALddx + Akddx(obj.Adep{kuser},obj.Adep{kuser}); ... need other way

      %%Akddx_expand=sparse(obj.Nx+obj.NYnnz,obj.Nx+obj.NYnnz);
      %Akddx_expand(obj.Adep{kuser},obj.Adep{kuser})=Akddx;
      %%Akddx_expand(Adep,Adep)=Akddx;
      %ALddx=ALddx + Akddx_expand;
      Akddx_expand = sparse(Akddx_row,Akddx_col,Akddx_val,obj.Nx+obj.NYnnz,obj.Nx+obj.NYnnz,lAdep*lAdep);
      Akddx_expand = Akddx_expand + tril(Akddx_expand,-1)';
      if (isempty(Akddx_lagr))
        %Akddx_lagr = sparse(Akddx_row,Akddx_col,Akddx_val2,obj.Nx+obj.NYnnz,obj.Nx+obj.NYnnz,lAdep*lAdep);
        %Akddx_lagr = Akddx_lagr + tril(Akddx_lagr,-1)';
        ALddx=ALddx + Akddx_expand;
        %ALddx=ALddx + Akddx_expand + Akddx_lagr;
      else
        ALddx=ALddx + Akddx_expand + Akddx_lagr;
      end
      % + add 2nd derivatives
    end
    end

    % store user's data back in the object if it got changed
    obj.userdata=userdata;
    
    % update ticker
    obj.ALddx = ALddx;
    obj.ALddxtck = obj.ticker;

    % update stats
    obj.stats_ncall_alddx = obj.stats_ncall_alddx + 1;
    obj.stats_time_alddx = obj.stats_time_alddx + cputime - starttime;

  end

