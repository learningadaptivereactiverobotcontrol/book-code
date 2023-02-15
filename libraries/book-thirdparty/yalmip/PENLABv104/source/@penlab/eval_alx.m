% Evaluate Augmented Lagrangian at the current 'xall' point with the current
% penalty parameters and Lagrangian multipliers. The result will be stored
% in 'obj.ALx' and its ticker updated. All equalities 'obj.eqx' get updated
% as well. Return nonzero if evaluation failed (matrix not pos def, etc.) and 
% in this case, the value ALx will be set to Inf.
function [status] = eval_alx(obj)
  
  % TODO ... set status when cannot evaluate or something like that
  status=0;

  % work only if the value is out of date
  if (obj.ALxtck < obj.ticker)
    starttime = cputime;
    
    % create local copies of obj.x,obj.Y to avoid checking repetitively if they
    % are 'up-to-date' with obj.xall, they get used many times in calls to
    % user's functions
    x=obj.x;
    Y=obj.Y;
    % same for obj.userdata, it might be quite expensive to get&store it in
    % the object
    userdata=obj.userdata;

    [fx, userdata] = obj.objfun(x, Y, userdata);
    obj.objx=fx;

    % function inequal & equal
    %ineqx = zeros(obj.Nineq,1);   % probably not necessary, rather = []?
    ineqx=[];
    %obj.eqx = zeros(obj.Neq,1);
    if (obj.NgNLN + obj.NgLIN>0)
      [gx, userdata] = obj.confun(x, Y, userdata);
      ineqx = obj.ineqshift + obj.ineqmlt .* gx(obj.ineqmap);
      obj.ineqx = ineqx;
      obj.eqx = obj.eqshift + gx(obj.eqmap);
    end
    
    % box constraints
    xboxx = [];
    if (obj.Nxbox>0)
      xboxx = obj.xboxshift + obj.xboxmlt .* obj.xall(obj.xboxmap);
      obj.xboxx=xboxx;
    end

    ALx=fx;

    %  inequalitites: phi or bar?
    %  L = L + u'*(p.*phi2(g./p));
    %  L = L + u(i)*p(i)*phi2(g(i)/p(i));

    % box constraints
    ind=obj.xboxindbar;
    if (~isempty(ind))
      ALx=ALx + obj.uxbox(ind)'*(obj.pxbox(ind).*obj.phibar(xboxx(ind)));
    end

    ind=obj.xboxindphi;
    if (~isempty(ind))
      ALx=ALx + obj.uxbox(ind)'*(obj.pxbox(ind).*obj.phi2(xboxx(ind)./obj.pxbox(ind)));
    end

    % function inequalitites
    %ind=obj.ineqindbar;
    %if (~isempty(ind))
    %  ALx=ALx + obj.uineq(ind)'*(obj.pineq(ind).*phibar(ineqx(ind)));
    %end

    ind=obj.ineqindphi;
    if (~isempty(ind))
      ALx=ALx + obj.uineq(ind)'*(obj.pineq(ind).*obj.phi2(ineqx(ind)./obj.pineq(ind)));
    end

    % (function) equalities
    if (obj.Neq>0)
      ALx = ALx + obj.ueq'*obj.eqx;
    end

    % matrix variable - log barrier (strict feasibility)
    if ~isempty(obj.Yboxindbar)
    for k=obj.Yboxindbar
      % convert the matrix box constraint to the form:   +/-Y +/-bound >=0
      pkx=obj.PYbox(k);
      Ykx = Y{obj.Yboxmap(k)};
      Akx=-obj.Yboxshift(k)*speye(size(Ykx)) - obj.Yboxmlt(k)*Ykx;
      % check that it is pos. def.
      [R,iii]=chol(Akx);
      if (iii~=0)
        ALx=Inf;
        % jump out and even avoid anything other cycle
        break;  
      end

      % compute penalty term: -p*log(det(A)) = -p*log( det(R)^2 ) = 
      %    = -2*p*log( prod(diag(R)) ) = -2*p*sum(log( diag(R) ))
      ALx = ALx -2*pkx*sum(log(diag(R)));

    end
    end


    % matrix variable - pen/bar
    if ~isempty(obj.Yboxindphi)
    for k=obj.Yboxindphi
      pkx=obj.PYbox(k);            %2*p(sdpdata.Ng+k);
      Ykx = Y{obj.Yboxmap(k)};
      Akx=obj.Yboxshift(k)*speye(size(Ykx)) + obj.Yboxmlt(k)*Ykx;
      umatk=obj.UYbox{k};

      %Z=(pkx*speye(size(Akx))-Akx);
      Z=(pkx*speye(size(Akx))-Akx);
      [R,iii]=chol(Z);   % to double check that it is pos. def!
      if (iii~=0)
        ALx=Inf;
        % jump out and even avoid anything other cycle
        break;  
      end
        
      invZ=inv(Z);
      % ... TODO ...
      ALx = ALx + trace(pkx^2*umatk*invZ-pkx*umatk);
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
      [R,iii]=chol(Z);   % to double check that it is pos. def!
      if (iii~=0)
        ALx=Inf;
        break;
      end
      invZ=inv(Z);
      % ... TODO ...
      %ALx = ALx + trace(pkx^2*umatk*invZ-pkx*umatk);
      ALx = ALx + pkx^2*(umatk(:)'*invZ(:))-pkx*trace(umatk);
    end
    end

    % store user's data back in the object if it got changed
    obj.userdata=userdata;

    % update ticker
    obj.ALx = ALx;
    obj.ALxtck = obj.ticker;

    % update stats
    obj.stats_ncall_alx = obj.stats_ncall_alx + 1;
    obj.stats_time_alx = obj.stats_time_alx + cputime - starttime;
  end

