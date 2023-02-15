function sdpdata = readsdpa(filename); 
% READSDPA - Read a linear SDP problem from a Sparse SDPA file
%  separate the linear constraint matrix, return
% the problem in the following structure:
%  
% Elements of the structure
%   name ... filename of the input file
%   Nx ..... number of primal variables
%   Na ..... number of linear matrix inequalities (or diagonal blocks of the
%            matrix constraint)
%   Ng ..... number of linear inequalitites
%   B ...... matrix defining the linear inequality constraints Bx<=d
%            dimension Ng x Nx
%   d ...... rhs for linear constraints
%   c ...... dim (Nx,1), coefficients of the linear objective function
%   NaDims . vector of sizes of matrix constraints (diagonal blocks)
%   A ...... cell array (matrix) of A{k,l} for k=1,...,Na matrix constraint
%            for l=1 ~ absolute term, l=2..Nx+1 coeficient matrices
%            (some of them might be empty)
%   Adep ... dependency list for each matrix constraint
%
% See also sdp_define, pen2bmi

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 4 Dec 2013

  sdpdata=[];

  % open file
  fid=fopen(filename,'r');
  if (fid==-1)
    error(sprintf('Cannot open file "%s"',filename));
  end

  nline=0;
  phase=0;
  nx=0;
  nblocks=0;
  msizes=[];
  c=[];
  nentries=0;
  maxentries=0;
  alldata=[];
  while 1
    line=fgetl(fid);
    nline=nline+1;
    if (~ischar(line))
      % end of file
      break;
    end
    %fprintf('line %d: >>%s<<\n',nline,line);

    % skip comments or empty lines
    if (isempty(line) || line(1)=='*' || line(1) =='"')
      continue;
    end

    switch (phase)
    case 0
      % expecting number of variables
      [nx,count]=sscanf(line,'%d',1);
      if (count~=1)
        error(sprintf('Line %d, cannot read number of variables.',nline));
      elseif (nx<=0)
        error(sprintf('Line %d, wrong number of variables.',nline));
      end
      phase=phase+1;
      %nx

    case 1
      % expecting number of matrix constraints (=blocks)
      [nblocks,count]=sscanf(line,'%d',1);
      if (count~=1)
        error(sprintf('Line %d, cannot read number of blocks.',nline));
      elseif (nblocks<=0)
        error(sprintf('Line %d, wrong number of blocks.',nline));
      end
      phase=phase+1;
      %nblocks

    case 2
      % expecting block sizes which might be separated by foreign
      % characters such as {,},(,),... Add space at the beginning
      % of the line in case there is nothing
      [msizes,count]=sscanf([' ' line],'%*[^0-9+-]%d',nblocks);
      if (count~=nblocks)
        error(sprintf('Line %d, cannot read block sizes.',nline));
      elseif (any(msizes==0))
        error(sprintf('Line %d, block sizes are incorrect.',nline));
      end
      phase=phase+1;
      %msizes

    case 3
      % expecting the objective vector perhaps mixed with some garbage
      [c,count]=sscanf([' ' line],'%*[^0-9eE.+-]%lg',nx);
      if (count~=nx)
        error(sprintf('Line %d, cannot read the objective vector.',nline));
      end
      phase=phase+1;
      %c

    case 4
      % expecting a data line
      [data,count]=sscanf(line,'%i %i %i %i %lg',5);
      if (count~=5)
        error(sprintf('Line %d, cannot read the data line.',nline));
      end
      nentries=nentries+1;
      if (nentries>maxentries)
        alldata=[alldata,zeros(5,2500)];
        maxentries=maxentries+2500;
      end
      alldata(:,nentries)=data;
      %alldata(:,1:nentries)

    end

  end

  % close file
  fclose(fid);

  if (nx<=0 || nblocks<=0 || isempty(msizes) || isempty(c) || nentries<=0)
    error('The file seems to be incomplete.');
  end

  % remove empty entries
  alldata=alldata(:,1:nentries);

  % check the correctness of the data lines
  if (any(alldata(1,:)<0 | alldata(1,:)>nx))
    error('Some of the data lines have matrix_number out of range.');
  end
  if (any(alldata(2,:)<1 | alldata(2,:)>nblocks))
    error('Some of the data lines have block_number out of range.');
  end

  % extract the linear constraints
  % turn 1-size matrix blocks into linear constraints
  idx=find(msizes==1);
  msizes(idx)=-1;
  linblk=find(msizes<0);
  nlin=sum(abs(msizes(linblk)));
  nnzlin=length(find(msizes(alldata(2,:))<0));
  % accummulate data into B matrix
  B=sparse([],[],[],nx,nlin,nnzlin);
  d=zeros(nlin,1);
  ng=0;    % no of the constraint written so far
  for iblk=linblk'
    dim=-msizes(iblk);
    idxentries=find(alldata(2,:)==iblk);
    thisblock=alldata(:,idxentries);
    if (any(thisblock(3,:)<1 | thisblock(3,:)>dim | ...
      thisblock(3,:)~=thisblock(4,:)))
      error(sprintf('Diagonal block %d have indices nondiag. or out of range elements.',iblk));
    end
    % extract RHS
    idx=find(thisblock(1,:)==0);
    if (~isempty(idx))
      d(ng+thisblock(3,idx))=thisblock(5,idx);
    end
    % extract linear constraints bodies
    idx=find(thisblock(1,:)>0);
    if (~isempty(idx))
      B(:,ng+1:ng+dim)=sparse(thisblock(1,idx),thisblock(3,idx),thisblock(5,idx),nx,dim);
    end
    ng=ng+dim;
  end
  %d
  %spy(B)
  
  % extract matrix constraints
  matblk=find(msizes>0);
  na=length(matblk);
  A=cell(na,nx+1);
  for iblk=matblk'
    dim=msizes(iblk);
    idxentries=find(alldata(2,:)==iblk);
    thisblock=alldata(:,idxentries);
    if (any(thisblock(3,:)<1 | thisblock(3,:)>dim | thisblock(4,:)<1 | ...
      thisblock(4,:)>dim))
      error(sprintf('Block %d have indices not matching its dim=%d.',iblk,dim));
    end
    % if i>j --> lower triangle which is not allowed
    if (any(thisblock(3,:)>thisblock(4,:)))
      error(sprintf('Block %d have elements outside upper triangle.',iblk));
    end
    % extract each of the matrices in this block
    for i=0:nx
      idx=find(thisblock(1,:)==i);
      if (isempty(idx))
        A{iblk,i+1}=sparse(dim,dim);
      else
        M=sparse(thisblock(3,idx),thisblock(4,idx),thisblock(5,idx),dim,dim);
        A{iblk,i+1}=M+triu(M,1)';
      end
    end
  end

  % put everything together into one structure
  sdpdata.name=filename;
  sdpdata.Nx=nx;
  sdpdata.Na=na;
  sdpdata.Ng=nlin;
  sdpdata.B=B';
  sdpdata.d=d;
  sdpdata.c=c;
  sdpdata.NaDims=msizes;
  sdpdata.A=A;


%  if 0


% create dependency table, Adep(k) = vector of all x indices which Ak depends on
sdpdata.Adep=cell(sdpdata.Na,1);
for k=1:sdpdata.Na
  list=[];
  for i=2:nx+1
    if (~isempty(sdpdata.A{k,i}) && nnz(sdpdata.A{k,i})>0)
      list=[list,i-1];
    end
  end
  sdpdata.Adep{k}=list;
end


end

