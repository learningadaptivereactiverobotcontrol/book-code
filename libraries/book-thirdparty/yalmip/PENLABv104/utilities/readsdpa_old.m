function sdpdata=readsdpa(filename); 
% read the file & separate the linear constraint matrix, return
% the problem in the following structure:
%  
%function [mDIM,nBLOCK,bLOCKsTRUCT,c,F]=readsdpa(filename); 
%
% Read a problem in SDPA sparse format.
%
% [mDIM,nBLOCK,bLOCKsTRUCT,c,F] = read_data(fname)
%
% <INPUT>
% - filename: string; filename of the SDP data with SDPA foramt.
%
% <OUTPUT>
% - mDIM       : integer; number of primal variables
% - nBLOCK     : integer; number of blocks of F
% - bLOCKsTRUCT: vector; represetns the block structure of F
% - c          : vector; coefficient vector
% - F          : cell array; coefficient matrices
%

% This file is a component of SDPA
% edited by JF 2011
%
% Copyright (C) 2004 SDPA Project
% 
% This program is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 2 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program; if not, write to the Free Software
% Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
%
% SDPA-M: $Revision: 6.2 $
% $Id: read_data.m,v 6.2 2005/05/28 02:36:40 drophead Exp $

% check the validity of the arguments
if ( nargin ~= 1 | ( nargin == 1 & ~isstr(filename) ) )
  error('input argument must be a filename');
end

sdpdata=[];

% identify whether a file is sparse format or not.
bsparse=0;
len=length(filename);
if len >= 2
  str=filename(end-1:end);
  if strncmp(str,'-s',2) 
    bsparse=1;
  end
end

fid=fopen(filename,'r');
if fid == -1
  error(sprintf('Cannot open %s',filename));
end

% skip comment and after it, read a number of decision variables (mDIM)
while 1
  str=fgetl(fid);
  if( str(1)~='*' & str(1) ~='"' )
    mDIM=sscanf(str,'%d',1);
    break;
  end
end
%disp(sprintf('mDIM=%d',mDIM));

% read a number of blocks (nBLOCK)
nBLOCK=fscanf(fid,'%d',1);
%disp(sprintf('nBLOCK=%d',nBLOCK));

% read each size of blocks (bLOCKsTRUCT)
bLOCKsTRUCT=zeros(nBLOCK,1);
for idx=1:nBLOCK
  bLOCKsTRUCT(idx)=fscanf(fid,'%*[^0-9+-]%d',1);
  %if bLOCKsTRUCT(idx) == 1
  %  bLOCKsTRUCT(idx) = -1;
  %end
end

% read cost vector (c)
c=zeros(mDIM,1);
for idx=1:mDIM
  c(idx)=fscanf(fid,'%*[^0-9+-]%lg',1);
end

% read coefficient matrices (F)
F=cell(nBLOCK,mDIM+1);

if bsparse
  % sparse format case
  while 1
    [k,cnt]=fscanf(fid,'%*[^0-9+-]%d',1);
    [l,cnt]=fscanf(fid,'%*[^0-9+-]%d',1);
    [i,cnt]=fscanf(fid,'%*[^0-9+-]%d',1);
    [j,cnt]=fscanf(fid,'%*[^0-9+-]%d',1);
    [value,cnt]=fscanf(fid,'%*[^0-9+-]%lg',1);
    if cnt
      if isempty(F{l,k+1})
        bsize=abs(bLOCKsTRUCT(l));
        if bLOCKsTRUCT(l) < 0
          F{l,k+1}=sparse(zeros(bsize,1));
        else
          F{l,k+1}=sparse(zeros(bsize));
        end
      end
      if bLOCKsTRUCT(l) < 0
        F{l,k+1}(i)=value;
      else
        if i < j
          F{l,k+1}(i,j)=value;
          F{l,k+1}(j,i)=value;
        elseif i == j
          F{l,k+1}(i,j)=value;
        end
      end
    else 
      break;
    end
  end
else
  % dense format case
  for k=1:mDIM+1
    for l=1:nBLOCK
      bsize=abs(bLOCKsTRUCT(l));
      if bLOCKsTRUCT(l) > 0
        F{l,k}=zeros(bsize);
        for i=1:bsize
          for j=1:bsize
            [value,cnt]=fscanf(fid,'%*[^0-9+-]%lg',1);
            if cnt
              F{l,k}(i,j)=value;
            else
              error(sprintf('Failed to read an element at %d %d %d %d'),...
            k-1,l,i,j);
            end
          end
        end
      else
        F{l,k}=zeros(bsize,1);
        for i=1:bsize
          [value,cnt]=fscanf(fid,'%*[^0-9+-]%lg',1);
          if cnt
            F{l,k}(i)=value;
          else
            error(sprintf('Failed to read an element at %d %d %d %d'),...
              k-1,l,i,i);
          end
        end
      end
    end
  end
end
fclose(fid);

% How to treat 1-dim blocks - exclude into the linear inequality block?
%linblk=find(bLOCKsTRUCT<0 | bLOCKsTRUCT==1);
linblk=find(bLOCKsTRUCT<0);
matr=setdiff(1:nBLOCK,linblk);
sdpdata.Ng=sum(abs(bLOCKsTRUCT(linblk)));

%linblk
%matr

% this might not work if linblk has more than 1 element
% do blocks one by one + perhaps use vertcat (instead of horzcat [])
%B=[F{linblk,2:end}];
%d=[F{linblk,1}];

B=[]; d=[];
% still doesn't work (see truss1) - some of the matrices (columns in B) are 
% empty instead of zeros -> won't concatenate the columns correctly
% ==> modify F and substitute empty matrices by zero matrices
if (~isempty(linblk))
  for k=linblk
    sz=abs(bLOCKsTRUCT(k));
    for i=1:mDIM+1
      if (isempty(F{k,i}))
        F{k,i} = sparse(sz, 1); % in linblk ==> it is just a vector, not a matrix
      end
    end
  end
  for k=linblk
    Bk=[F{k,2:end}];   % should be a matrix dim_of_kth_block x mDIM
    dk=[F{k,1}];       % should be a vector dim_of_kth_block x 1
    % check dimensions if concatenated properly
    if (size(Bk,1)~=abs(bLOCKsTRUCT(k)) || size(Bk,2)~=mDIM)
      disp(sprintf('ERR: wrong dimensions of Bk, k=%i: %i %i',k,size(Bk)))
    end
    if (size(dk,1)~=abs(bLOCKsTRUCT(k)) || size(dk,2)~=1)
      disp(sprintf('ERR: wrong dimensions of dk, k=%i: %i %i',k,size(dk)))
    end
    % vertical concatenation
    B=[B;Bk];
    d=[d;dk];
  end
end

%size(B)
%size(d)

% apply the same fix for the first blocks ("absolute term"), the others are covered by Adep
for k=matr
  sz=bLOCKsTRUCT(k);
  if (isempty(F{k,1}))
    F{k,1}=sparse(sz,sz);
  end
end

% probably won't work as F{k,i}... if F{k,i} is empty it won't add a column :(
% better [F{k,:}]
%B=[];
%for k=linblk
%  D=[];
%  for i=2,mDIM+1
%    D=[D, F{k,i}]
%  end
%  B=[B;D]
%end

sdpdata.name=filename;
sdpdata.Nx=mDIM;
sdpdata.Na=length(matr);  %nBLOCK;
sdpdata.B=B;
sdpdata.d=d;
sdpdata.c=c;
sdpdata.NaDims=bLOCKsTRUCT(matr);
sdpdata.A=F(matr,:);

% create dependency table, Adep(k) = vector of all x indices which Ak depends on
sdpdata.Adep=cell(sdpdata.Na,1);
for k=1:sdpdata.Na
  list=[];
  for i=2:mDIM+1
    if (~isempty(sdpdata.A{k,i}) && nnz(sdpdata.A{k,i})>0)
      list=[list,i-1];
    end
  end
  sdpdata.Adep{k}=list;
end

sdpdata.origF=F;
sdpdata.origNaDims=bLOCKsTRUCT;

% End of File
