clear m n n1 BI xy maska ijk DELTA  

m=par.m; n=par.n; n1=par.n1; BI=par.BI; xy=par.xy;
  maska=par.maska; ijk=par.ijk;
  DELTA=par.DELTA;
  
  x = 425*ones(m,1);
  %x=penm.x;
  
  BI = sparse(BI(:,maska));
  %par.BI = BI;
  
  par.f=par.f;
  ff=par.f; 
  
  % PARAMETERS TO BE CHANGED MANUALLY
  compl = 1.0; par.cmp=compl;
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  for i=1:m
    Ahelp = BI(i,:)'*BI(i,:);
    %A{1,i+1} = [0 sparse(1,n1); sparse(n1,1) Ahelp(maska,maska)];
    A{1,i+1} = Ahelp;
  end
  A{1,1} = [compl -ff'; -ff sparse(n1,n1)];
  
      K=zeros(n1,n1);
    for i=1:m
        K=K+x(i)*A{1,i+1};
    end
    
    ainvf = K\ff;
    G=zeros(n1,n1);
    for i=1:m
        G = G - x(i)*BI(i,:)*ainvf*reshape(DELTA(:,i),n1,n1);
    end
   
    ee=eig(K,-G);
    min(ee(ee>0))