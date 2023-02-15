%%*******************************************************************
% SED2PEN converts problem data in SeDuMi format to expanded SDPA format.      
%
% [pen] = sdpa2pen(fname)
%
% Input: fname...name of the file containing SDP data in SDPA format
%
% Output: pen...structure with data arrays in PENLIB format
%
% Copyright (c) 2002 by M. Kocvara and M. Stingl
% Version 02/12/2002
%******************************************************************

  function pen=sed2pen(fname, compressed);
%%
%  First, load the matlab file containing At, c, b, and K
%%

clear pen;

  K.q = [];
  K.l = [];
  A = 0;
  At = 0;
  AAk = 0;
  ii = 0;
  ss = 0;
  fname2 = strcat(fname,'.dat-s');
  
  compressed=0;
  if (compressed == 1);
     fprintf('** Unzip Problem. \n');
     unix(['gunzip ', fname,'.mat.gz']);
  elseif (compressed == 2);
     unix(['uncompress ', fname,'.mat.Z']);
  end
  if exist([fname,'.mat']) | exist(fname) 
     eval(['load ', fname]);
  else
     fprintf('** Problem not found, please specify the correct path. \n');
     return;
  end
  if (compressed == 1)
     unix(['gzip ', fname,'.mat']);
  elseif (compressed == 2)
     unix(['compress ', fname,'.mat']);
  end

%%
  if (size(c,1) == 1), c = c'; end;
  if (size(b,1) == 1), b = b'; end;
  if (At == 0), At = A'; end;  clear A; 
%  if (At == 0), At = A'; end;  clear A;
  [nn,mm] = size(At); 
  if length(b)==nn, At=At'; [nn,mm] = size(At); end
  if (max(size(c)) == 1); c = c*ones(nn,1); end; 
  
  if ~isfield(K,'l'); K.l = 0; end  
  if ~isfield(K,'f'); K.f = 0; end 
  %if ~isfield(K,'q'); K.q = 0; end
  if ~isfield(K,'s'); K.s = 0; end
  if isempty(K.l) | K.l == 0; K.l = 0; end;
  if isempty(K.f) | K.f == 0; K.f = 0; end;
  %if sum(K.q) == 0 | isempty(K.q); K.q = 0; end
  if sum(K.s) == 0 | isempty(K.s); K.s = 0; end 
  m = length(b);
  pen.vars = m;
  %fprintf(fid,'%d\n',m);
  llen = 0;
  offset1 = 0;
  offset2 = 0;
  if(K.l > 0); llen = llen + length(K.l); end;
  offset1 = llen; 
  %if(K.q > 0); llen = llen + length(K.q); end;
  offset2 = llen;
  if(K.s > 0); llen = llen + length(K.s); end;
  %fprintf(fid,'%d\n',llen);
  
  pen.fobj = full(b);
  
  rowidx = 0;  idxblk = 0;  aidx = 0; bk1 = 1; hidx = 0;
  
  pen.constr = 0; pen.ci = 0; pen.bi_dim = 0; pen.bi_idx = 0; pen.bi_val = 0;
   if ~(K.l == 0) | ~(K.f == 0)
      len = K.l + K.f; 
      pen.constr = len + K.f;
      pen.ci = c(1:len);
      if (K.l == 0)
         if (K.f ~= 0), pen.ci = full([c(1:K.f);-c(1:K.f)]);end;
      else
         if (K.f ~= 0), pen.ci = full([c(1:K.f);-c(1:K.f)';c(K.f+1,len)]);end;
      end   
      idxblk = idxblk + 1; 
      if(K.f == 0)   
         for k = 1:len
         	Atmp = At(rowidx+k,:); 
            [ii,jj,ss] = find(Atmp);
            pen.bi_dim(k) = nnz(Atmp);
            pen.bi_idx(bk1:bk1+nnz(Atmp)-1) = jj-1;
            pen.bi_val(bk1:bk1+nnz(Atmp)-1) = -ss;
            bk1 = bk1 + nnz(Atmp);
	        % for kk = 1:nnz(Atmp)
           %    fprintf(fid,'%5d %5d %5d %5d %.16f\n',jj(kk),1,k,k,-ss(kk));
           % end
         end
      else      
         for k = 1:K.f
         	Atmp = At(rowidx+k,:); 
            [ii,jj,ss] = find(Atmp);
            pen.bi_dim(k) = nnz(Atmp);
            pen.bi_idx(bk1:bk1+nnz(Atmp)-1) = jj-1;
            pen.bi_val(bk1:bk1+nnz(Atmp)-1) = -ss;
            bk1 = bk1 + nnz(Atmp);
	         % for kk = 1:nnz(Atmp)
            %    fprintf(fid,'%5d %5d %5d %5d %.16f\n',jj(kk),1,k,k,-ss(kk));
            % end
         end
         for k = 1:K.f
         	Atmp = At(rowidx+k,:); 
            [ii,jj,ss] = find(Atmp);
            pen.bi_dim(K.f+k) = nnz(Atmp);
            pen.bi_idx(bk1:bk1+nnz(Atmp)-1) = jj-1;
            pen.bi_val(bk1:bk1+nnz(Atmp)-1) = ss;
            bk1 = bk1 + nnz(Atmp);
	         % for kk = 1:nnz(Atmp)
            %    fprintf(fid,'%5d %5d %5d %5d %.16f\n',jj(kk),1,k,k,-ss(kk));
            % end
         end
         if (K.l ~= 0)
            for k = K.f+1,len
         	   Atmp = At(rowidx+k,:); 
               [ii,jj,ss] = find(Atmp);
               pen.bi_dim(K.f+k) = nnz(Atmp);
               pen.bi_idx(bk1:bk1+nnz(Atmp)-1) = jj-1;
               pen.bi_val(bk1:bk1+nnz(Atmp)-1) = -ss;
               bk1 = bk1 + nnz(Atmp);
	            % for kk = 1:nnz(Atmp)
               %    fprintf(fid,'%5d %5d %5d %5d %.16f\n',jj(kk),1,k,k,-ss(kk));
               % end
            end
         end
      end  
      rowidx = rowidx + len; 
   end

   if ~(K.s == 0) 
      blksize = K.s;  
      if size(blksize,2) == 1; blksize = blksize'; end
      pen.ai_dim = zeros(1,length(blksize));
      pen.msizes = blksize;
      pen.mconstr = length(blksize);
      blknnz = [0 cumsum(blksize.*blksize)];   
      for p = 1:length(blksize)
        	 idxblk = idxblk + 1; 
          n = blksize(p); 
          Ctmp = c(rowidx+blknnz(p)+[1:n*n]);
          nn = nnz(Ctmp);
          if(nn>0)
             aidx = aidx + 1;
             pen.ai_dim(p) = 1;
             pen.ai_idx(aidx) = 0;
             [ii,jj,ss] = find(Ctmp);
             iinz = 0;
             for kk = 1:nn
                idxi = fix(((ii(kk)-1)./n)+1);
                idxj = mod(ii(kk)-1,n)+1;
                if ~(idxi > idxj)
                   hidx = hidx + 1;   
                   iinz = iinz + 1;
                   pen.ai_val(hidx) = -ss(kk);
                   pen.ai_row(hidx) = idxi-1;
                   pen.ai_col(hidx) = idxj-1;
                   %fprintf(fid,'%5d %5d %5d %5d %.16f\n',0,p+offset2,idxi,idxj,-ss(kk));
                end              
             end
             pen.ai_nzs(aidx) = iinz;
          end  
       
          Atmp = At(rowidx+blknnz(p)+[1:n*n],:);
          for k = 1:m
             AAk = Atmp(:,k);
             nn = nnz(AAk);
             if(nn>0)
                aidx = aidx + 1;
                pen.ai_dim(p) = pen.ai_dim(p) + 1;
                pen.ai_idx(aidx) = k;
                iinz = 0;
                [ii,jj,ss] = find(AAk);
                for kk = 1:nn
                   idxi = fix(((ii(kk)-1)./n)+1);
                   idxj = mod(ii(kk)-1,n)+1;
                   if ~(idxi > idxj)
                      hidx = hidx + 1;  
                      iinz = iinz + 1;
                      pen.ai_val(hidx) = -ss(kk);
                      pen.ai_row(hidx) = idxi-1;
                      pen.ai_col(hidx) = idxj-1;                       
                      %fprintf(fid,'%5d %5d %5d %5d %.16f\n',k,p+offset2,idxi,idxj,-ss(kk));
                   end
                end
                pen.ai_nzs(aidx) = iinz;             
             end
          end
       end
    end   
%%
pen.x0 = zeros(1,pen.vars);

%pen.ioptions = [0];

%pen.foptions = [0];


pen.ioptions = [1 50 100 2 0 1 0 0 3 2 0 0 2 1 1];

pen.foptions = [1 0.7 0.1 1.0E-4 1.0E-7 1.0E-14 1.0E-2 1.0 0.5 1.0 1.0e-6 0.05];
