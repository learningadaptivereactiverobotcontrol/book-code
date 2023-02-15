function []=sdpastats(filename)
% SDPASTATS gets some statistics (& vision) how the given SDPA problem looks
% for example:
%    sdpastats('datafiles/arch0.dat-s');

% This file is a part of PENLAB package distributed under GPLv3 license
% Copyright (c) 2013 by  J. Fiala, M. Kocvara, M. Stingl
% Last Modified: 27 Nov 2013

  % read the file in
  disp(['Reading the input file: ',filename]);
  tic;
  sdpdata=readsdpa(filename);
  toc
  Nx=sdpdata.Nx;
  Na=sdpdata.Na;

  % create nonzero pattern of all matrices in A
  A=cell(Na,1);
  Ader=cell(Na,1);
  Adim=zeros(Na,1);
  Adep=zeros(Na,1);
  dim=0;
  rowall=[];
  colall=[];
  for k=1:Na
    Akx=spones(sdpdata.A{k,1});
    Ader{k}=zeros(5,1);
    for i=sdpdata.Adep{k}
      Akx = Akx + spones(sdpdata.A{k,i+1});
      Ader{k}=sparsetype(Ader{k},sdpdata.A{k,i+1});
    end
    %figure;spy(Akx);
    n=size(Akx,1);
    A{k}=Akx;
    Adep(k)=length(sdpdata.Adep{k});
    Adim(k)=n;
    [row,col]=find(Akx);
    rowall=[rowall;row+dim];
    colall=[colall;col+dim];
    dim=dim+n;
  end

  Aall=sparse(rowall,colall,ones(size(rowall)),dim,dim);
  %figure;
  spy(Aall);

  fprintf('Nx = %5i,  NA = %5i\n',Nx,Na);
  fprintf('  k:   dim sparsity  #dep    %%dep  |  <5%% <10%% <25%% <50%% >50%%\n');
  for k=1:Na
    sparsity=nnz(A{k})/Adim(k)/Adim(k)*100;
    pct=Adep(k)/Nx*100;
    fprintf('%3i: %5i  %5.1f%%  %5i  %5.1f%%  | %4i %4i %4i %4i %4i\n',k,Adim(k),sparsity,Adep(k),pct,Ader{k});
  end

end

%%%%%
% count which 'category' the given matrix falls into w.r.t. its density
% categoriesL: 5%, 10%, 25%, 50%, >50%
function [der]=sparsetype(der,A)
  nz=nnz(A);
  n=size(A,1);
  nnzmax=n*n;
  if (nz<0.05*nnzmax)
    der(1)=der(1)+1;
  elseif (nz<0.1*nnzmax)
    der(2)=der(2)+1;
  elseif (nz<0.25*nnzmax)
    der(3)=der(3)+1;
  elseif (nz<0.5*nnzmax)
    der(4)=der(4)+1;
  else
    der(5)=der(5)+1;
  end
end

