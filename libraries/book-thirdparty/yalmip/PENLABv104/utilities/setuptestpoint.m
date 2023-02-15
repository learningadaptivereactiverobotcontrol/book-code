
function [alx,aldx,alddx] = setuptestpoint(obj, filename, type)
% SETUPTESTPOINT sets a point from original Pennon for comparison
% prob ... initialized object with the same problem as it is stored in DCF
% filename ... name (path) to a DCF (Data Container File) generated from
%   Pennon suit, so far PenSDP (for linear SDP stored in SDPA) or Pennon/Pennlp
%   for Ampl problems
% type ... what's the type (origin) of the DCF? either 'pennlp' or 'pensdp'

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  % make datatransfer quiet?
  global DCF_SILENT
  DCF_SILENT=1;

  if (strcmpi(type,'pennlp') || strcmpi(type,'nlp'))

    % unfortunately, it is impossible to identify content of the dcf file 
    % automatically ... so it is expected that all dcf files are in the same 
    % format (for now, xUPgHAuf)

    % read the data and check their dimensions
    x=datatransfer(filename,0);     % dim Nx
    pen_u=datatransfer(filename,1); % dim Nxbox+Nineq+Neq 
      % lagr. mlt. for all mixed together (box, NLN, LIN; NLN & LIN include eq)
      % order of transformed inequalities: lb<=g(x)<=ub should be the same
      % as here, i.e., lb-g(x)<=0 followed by g(x)-ub<=0
    pen_p=datatransfer(filename,2); % dtto as pen_u but for penalty parameters
    aldx=datatransfer(filename,3);  % gradient of augm. lagr.
    pen_H=datatransfer(filename,4); % either (1,1) block of KKT or the whole one
    pen_A=datatransfer(filename,5); % Jacobian of equalities if store separately
    ueq=datatransfer(filename,6);   % Lagr. mlt. for equalities
    alx=datatransfer(filename,7);   % value of augm. lagr. without penalization
      % of violation of equalitites (i.e. augm. lagr., not merit function)

    % check if the 'obj' looks compatible and copy all the data
    if (length(x)~=obj.Nx || obj.NYnnz~=0)
      error('DCF does not fit the number of variables.');
    end
    if (obj.NA~=0)
      error('Problem object is not a NLP (only) problem.');
    end
    if (length(pen_u)~=obj.Nxbox+obj.Nineq+obj.Neq)
      error('Problem object does not seem to be defined by the same NLP model');
    end
    if (length(pen_u)~=length(pen_p))
      error('Arrays U and P in DCF are of different size? Corrupted DCF?');
    end
    if (length(pen_u)~=length(obj.userdata.BEQUAL_M2))
      error('Mapping to separate ineq+eq is incompatible, different problem loaded?');
    end
    if (length(aldx)~=obj.Nx)
      error('DCF looks incompatible (because of aldx).');
    end
    if (length(ueq)~=obj.Neq)
      error('Different number of equalities.');
    end
    % check pen_H, pen_A ?

    % get only inequality multipliers and penalties
    u=pen_u(~obj.userdata.BEQUAL_M2);
    p=pen_p(~obj.userdata.BEQUAL_M2);

    % copy everything
    obj.xall=x;
    obj.uxbox=u(1:obj.Nxbox);
    obj.uineq=u(obj.Nxbox+1:end);
    obj.pxbox=p(1:obj.Nxbox);
    obj.pineq=p(obj.Nxbox+1:end);
    obj.ueq=ueq;
    
    % transform Hessian from eqltymode=3 (e.i., separate equality Jacobian)
    [pha,phb]=size(pen_H);
    if (obj.Neq~=0 && isempty(pen_A) && pha==obj.Nx+obj.Neq && phb==pha)
      disp('  ---Separating equalities from input data.---');
      disp(' ');
      pen_A = pen_H(1:obj.Nx,(obj.Nx+1):phb);
      pen_H = pen_H(1:obj.Nx,1:obj.Nx);
    %else
    %  warning('strange pen_H??');
    end

    % Do NOT trust to the dense hessian, only upper half is for sure complete
    if (~issparse(pen_H))
      disp('  ---pen_H is dense, symmetrizing it for sure---');
      disp(' ');
      pen_H_ut=triu(pen_H,1);
      %pen_H = pen_H.*triu(ones(obj.Nx,obj.Nx)) + pen_H_ut';
      pen_H = triu(pen_H) + pen_H_ut';
    end

    alddx=[pen_H, pen_A; pen_A', sparse(obj.Neq,obj.Neq)];

  elseif (strcmpi(type,'pensdp') || strcmpi(type,'sdp'))
    % reads/decodes the saved data structure from Pennon (linear) SDP
    % expect "xUPDfgHv" (default)

    % don't forget that the indices in datatransfer are 0-based!
    x=datatransfer(filename,0);  % dim Nx; x
    u=datatransfer(filename,1);  % dim Ng; lagr. multiplers for normal inequal.
    p=datatransfer(filename,2);  % dim Ng+Na; penalty params for ineq. & matr
      % really for both? Yes
    dense=datatransfer(filename,3); % density flag
    alx=datatransfer(filename,4); % merit function (AL + penalty for equal)
    aldx=datatransfer(filename,5);% dim Nx; gradient of AL
    alddx=datatransfer(filename,6);% dim Nx x Nx, might be sparse; hessian of AL
    % symetrize alddx if dense (perhaps even if sparse? ... Need to test)
    if (~issparse(alddx))
      % only upper triangle is stored
      alddx = alddx+triu(alddx,1)';
    end
    % matrix constraint lagrangian multipliers
    Na = datatransfer(filename,7); % N_MATRIX_CONSTR
    umat=cell(Na,1);
    for k=1:Na
      % either dense ... dim x dim or sparse (dim*(dim+1)/2) x 1
      % TODO careful, datatransfer expects double as the number of elements!
      % and iterating index is automatically int.
      umatk=datatransfer(filename,double(7+k));
      if (size(umatk,1)~=size(umatk,2))
        % Ak is sparse ==> unpack packed triangular format of umat
        % upper triangle, column order (1,2,3,...,dim nnz)
        % dim:  2*nnz=dim^2+dim -> 0=d^2+d-2nnz -> D=1+8nnz, d=(-1+-sqrt(D))/2
        nnz=size(umatk,1);
        dim=(-1+sqrt(1+8*nnz))/2;

        % get mapping for the packed format
        Mtmp=triu(ones(dim));
        [irow,icol]=find(Mtmp);
        utmp=sparse(irow,icol,umatk);
        umat{k}=full(utmp+triu(utmp,1)');
        %umat{k}(1:10,1:10)
        %umatk(1:20)
      else
        % dense, as it is
        umat{k}=umatk;
      end
    end
    
    % check if the 'obj' looks compatible and copy all the data
    if (length(x)~=obj.Nx || obj.NYnnz~=0)
      error('DCF does not fit the number of variables.');
    end
    if (Na~=obj.NA || obj.NANLN~=0)
      error('DCF has different number of matrix constraints.');
    end
    if (obj.Nxbox~=0 || obj.NgNLN~=0 || obj.Neq~=0)
      error('Problem object does not seem to be from SDPA');
    end
    if (length(u)~=obj.Nineq)
      error('DCF has a different number of linear inequalitites.');
    end

    obj.xall=x;
    obj.uineq=u;  % same order? should be ... only linear ineq.
    obj.pineq=p(1:obj.Nineq);
    for k=1:Na
      obj.UA{k}=umat{k};   % same order? ... should be
    end
    %obj.UA=umat;
    % in Pennon or my Matlab test there used to be 2*p in the computation
    % this version use only PA(k) ==> pass it here 2*
    obj.PA=2*p(obj.Nineq+1:end);


  else
    error('Unknown type of DCF file.');
  end

