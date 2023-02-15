function [Priors_P, Mu_P, Sigma_P ,Priors_V ,Mu_V ,Sigma_V]=Solver_MSE_Sym(Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V,Data_A,EPSILON,options)
d = size(Sigma_0_P,1)/2; %dimension of the model
K = size(Sigma_0_P,3);
if options.display
    str = 'iter';
else
    str = 'off';
end
 optNLP = optimset('Algorithm', 'interior-point','Diagnostics','on', 'LargeScale', 'on',...
        'GradObj', 'on', 'GradConstr', 'on','FinDiffType','central', ...
        'Display', 'iter', 'TolX',options.tol_stopping, 'TolFun', options.tol_stopping, 'TolCon', options.TolCon, ...
        'MaxFunEval', 20000, 'MaxIter', options.max_iter, 'DiffMinChange', ...
         options.tol_stopping, 'Hessian','bfgs','display',str,'AlwaysHonorConstraints','bounds'); %,'FinDiffType','central''DerivativeCheck','on' interior-point
 for i=1:K
    Sigma_0_P(1:d,1:d,i)=sqrtm(Sigma_0_P(1:d,1:d,i));
    Sigma_0_V(1:d,1:d,i)=sqrtm(Sigma_0_V(1:d,1:d,i));
 end
Sigma_input_P=Sigma_0_P(1:d,1:d,:);
Sigma_input_V=Sigma_0_V(1:d,1:d,:);
p0 = GMM_2_Parameters(Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V,d,K); 

[A,b,Aeq,beq,lb,ub,Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V]=Initilazation(p0,d,K);
% Sigma_0_P(1:d,1:d,:)=Sigma_input_P;
% Sigma_0_V(1:d,1:d,:)=Sigma_input_V;
p0 = GMM_2_Parameters(Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V,d,K);

 
obj_handle = @(p)obj((p),Data_A,d,K,Sigma_input_P,Sigma_input_V);    
 ctr_handle = @(p) ctr_eigenvalue((p),d,K,EPSILON,Sigma_input_P,Sigma_input_V);  
%  ctr_handle=[];

[popt,fval,exitflag] = fmincon(obj_handle, (p0),A,b,[],[],lb,ub,ctr_handle,optNLP);
% popt=transpose(popt);
%  optNLP = optimset('Algorithm', 'sqp','Diagnostics','on', 'LargeScale', 'on',...
%         'GradObj', 'on', 'GradConstr', 'on','FinDiffType','central', ...
%         'Display', 'iter', 'TolX',options.tol_stopping/100, 'TolFun', options.tol_stopping, 'TolCon', options.TolCon, ...
%         'MaxFunEval', 20000, 'MaxIter', options.max_iter, 'DiffMinChange', ...
%          options.tol_stopping, 'Hessian','bfgs','display',str,'AlwaysHonorConstraints','bounds','UseParallel','always'); 
% [popt,fval,exitflag] = fmincon(obj_handle, popt,A,b,Aeq,beq,lb,ub,ctr_handle,optNLP);

disp('The value of the objective function is:')
disp(fval)
Massage(exitflag); 
[Priors_P, Mu_P, Sigma_P ,Priors_V ,Mu_V ,Sigma_V] = shape_DS(popt,d,K);
for i=1:K
    Sigma_P(1:d,1:d,i)=Sigma_P(1:d,1:d,i)*Sigma_P(1:d,1:d,i)';
    Sigma_V(1:d,1:d,i)=Sigma_V(1:d,1:d,i)*Sigma_V(1:d,1:d,i)';
end
% Sigma_P(1:d,1:d,:)=Sigma_input_P;
% Sigma_V(1:d,1:d,:)=Sigma_input_V; 
[Priors_P, Mu_P, Sigma_P ,Priors_V, Mu_V ,Sigma_V]=Stabilizing_B_K(Priors_P,Mu_P,Sigma_P,Priors_V,Mu_V,Sigma_V,d,K);
Priors_P=abs(Priors_P)/sum(Priors_P);
Priors_V=abs(Priors_V)/sum(Priors_V);


function [A,b,Aeq,beq,lb,ub,Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V]=Initilazation(p0,d,K)
Aeq=[];
beq=[];
% A=zeros(2*(K+2*d*K+4*d*d*K),2);
Size_all=size(p0);
A=zeros(Size_all(1,2),4*K);
lb=[];
ub=[];

Aeq=zeros(2,Size_all(1,2));



[Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V] = shape_DS(p0,d,K);
Shift=(1+d+d*(3*d+1)/2)*K;

counter=1;
for i=1:K
    A(i,counter)=1;
    b(counter)=1;
    counter=counter+1;
end
for i=1:K
    A(i,counter)=-1;
    b(counter)=0;
    counter=counter+1;
end

for i=1:K
    A(i+Shift,counter)=1;
    b(counter)=1;
    counter=counter+1;
end
for i=1:K
    A(i+Shift,counter)=-1;
    b(counter)=0;
    counter=counter+1;
end
%     
% for o=1:K
% b(o)=1;
% b(o+K)=0;
% end


 A=transpose(A);

 for i=1:K
    Aeq(1,i)=1;
 end
 for i=1:K
    Aeq(2,i+Shift)=1;
 end
beq(1)=0;
beq(2)=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Priors_P, Mu_P, Sigma_P, Priors_V, Mu_V, Sigma_V]=Stabilizing_B_K(Priors_P,Mu_P,Sigma_P,Priors_V,Mu_V,Sigma_V,d,K)
for pointe_of_K=1:K
    Sigmainput=Sigma_P(1:d,1:d,pointe_of_K);
    SigmaInput_Output=Sigma_P((d+1):2*d,1:d,pointe_of_K);
    Muinput=Mu_P(1:d,pointe_of_K);
    Mu_P(d+1:2*d,pointe_of_K)=(SigmaInput_Output/(Sigmainput))*Muinput; 
end
for pointe_of_K=1:K
    Sigmainput=Sigma_V(1:d,1:d,pointe_of_K);
    SigmaInput_Output=Sigma_V((d+1):2*d,1:d,pointe_of_K);
    Muinput=Mu_V(1:d,pointe_of_K);
    Mu_V(d+1:2*d,pointe_of_K)=(SigmaInput_Output/(Sigmainput))*Muinput;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Massage(exitflag)
if exitflag==1
    
    disp('First-order optimality measure was less than options.TolFun, and maximum constraint violation was less than options.TolCon.');

elseif exitflag==0

    disp('Number of iterations exceeded options.MaxIter or number of function evaluations exceeded options.MaxFunEvals.');
    
elseif exitflag==-1    
    disp('Stopped by an output function or plot function.');
elseif exitflag==-2
    disp('No feasible point was found.');
elseif exitflag==-3
    disp('Change in the objective function value was less than options.TolFun and maximum constraint violation was less than options.TolCon.');
elseif exitflag==2
    disp('Change in x was less than options.TolX and maximum constraint violation was less than options.TolCon.');
elseif exitflag==3     
    disp('Change in the objective function value was less than options.TolFun and maximum constraint violation was less than options.TolCon.');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [J, DJ_Main]=obj(p,Data,d,K,Sigma_input_P,Sigma_input_V)

% This function computes the derivative of the likelihood objective function
% w.r.t. optimization parameters.
DJ_Main=0;
nData = size(Data,2);
% nData=1;
% Data=DataA(:,1:nData);
DJ=zeros(1,(K+d*K+d*(3*d+1)*K/2),nData);
DDX=zeros(d,nData);
Diff=zeros(d,nData);
[Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V] = shape_DS(p,d,K); 
Sigma_input_P=Sigma_0_P(1:d,1:d,:);
Sigma_input_V=Sigma_0_V(1:d,1:d,:);
for i=1:K
    Sigma_0_P(1:d,1:d,i)=Sigma_0_P(1:d,1:d,i)*Sigma_0_P(1:d,1:d,i)';
    Sigma_0_V(1:d,1:d,i)=Sigma_0_V(1:d,1:d,i)*Sigma_0_V(1:d,1:d,i)';
end
% Sigma_0_P(1:d,1:d,:)=Sigma_input_P;
%  Sigma_0_V(1:d,1:d,:)=Sigma_input_V;
[~,Mu_0_P,~,~,Mu_0_V,~]=Stabilizing_B_K(Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V,Mu_0_V,Sigma_0_V,d,K);
% i=1;
 X  = Data(1:d,:); 
 DX=Data(d+1:2*d,:); 
[DDX_X, beta_P,Numerator_P,Denominator_P,A_P] = GMR(Priors_0_P,Mu_0_P,Sigma_0_P, X, 1:d, d+1:2*d);
[DDX_V, beta_V,Numerator_V,Denominator_V,A_V] = GMR(Priors_0_V, Mu_0_V, Sigma_0_V, DX, 1:d, d+1:2*d);
DDX=DDX_X+DDX_V;
Diff=(DDX-Data(2*d+1:3*d,:));
% while i<nData 
% %     i
%     X  = Data(1:d,i); 
%     DX = Data(d+1:2*d,i);  
% %     [DDX(:,i),Denominator_P,Denominator_V,Numerator_P,Numerator_V,A_P,A_V]= SE_DS(Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V,DX,X);
%      Diff(:,i)=(DDX(:,i)-Data(2*d+1:3*d,i));   
%      if Denominator_P==0
%          Denominator_P=realmin;
%      end
%      if Denominator_V==0
%          Denominator_V=realmin;
%      end
%%% Derivatives  
%%% Position   
DJ_DO=transpose(Diff);
counter=1;
for j=1:K
    Dxi_PI=Diff_respect_to_pi(transpose(Priors_0_P), Mu_0_P, Sigma_0_P,X,d,K,j,Denominator_P,Numerator_P,A_P);
    DJ(1,counter,:)=diag(DJ_DO*Dxi_PI);
    counter=counter+1;
end
for j=1:K 
    for o=1:d
        Dxi_mu=Diff_respect_to_mu_input(transpose(Priors_0_P), Mu_0_P, Sigma_0_P,X,d,K,o,j,Denominator_P,Numerator_P,A_P);
        DJ(:,counter,:)=diag(DJ_DO*Dxi_mu);
        counter=counter+1; 
    end  
end  
for j=1:K 
    for colum=1:d 
        for row=colum:d
            Dxi_sigma=Diff_respect_to_sigmainput(transpose(Priors_0_P), Mu_0_P, Sigma_0_P,X,d,K,row,colum,j,Denominator_P,Numerator_P,A_P,Sigma_input_P(:,:,j));
            DJ(:,counter,:)=diag(DJ_DO*Dxi_sigma);
            counter=counter+1;   
         end
        for row=1:d
            Dxi_sigma=Diff_respect_to_sigmainputoutput(transpose(Priors_0_P), Mu_0_P, Sigma_0_P,X,d,K,row,colum,j,Denominator_P,Numerator_P);
            DJ(:,counter,:)=diag(DJ_DO*Dxi_sigma); 
            counter=counter+1;
        end
    end 
end
% 
% %% Velocity    
for j=1:K
    Dxi_PI=Diff_respect_to_pi(transpose(Priors_0_V), Mu_0_V, Sigma_0_V,DX,d,K,j,Denominator_V,Numerator_V,A_V);
    DJ(:,counter,:)=diag(DJ_DO*Dxi_PI);
    counter=counter+1;
end
for j=1:K
    for o=1:d
        Dxi_mu=Diff_respect_to_mu_input(transpose(Priors_0_V), Mu_0_V, Sigma_0_V,DX,d,K,o,j,Denominator_V,Numerator_V,A_V);
        DJ(:,counter,:)=diag(DJ_DO*Dxi_mu);
        counter=counter+1;
    end
end
for j=1:K
    for colum=1:d 
        for row=colum:d
            Dxi_sigma=Diff_respect_to_sigmainput(transpose(Priors_0_V), Mu_0_V, Sigma_0_V,DX,d,K,row,colum,j,Denominator_V,Numerator_V,A_V,Sigma_input_V(:,:,j));
% DJ(:,counter,:)=0; 
            DJ(:,counter,:)=diag(DJ_DO*Dxi_sigma);
            counter=counter+1;  
        end
        for row=1:d
            Dxi_sigma=Diff_respect_to_sigmainputoutput(transpose(Priors_0_V), Mu_0_V, Sigma_0_V,DX,d,K,row,colum,j,Denominator_V,Numerator_V);
            DJ(:,counter,:)=diag(DJ_DO*Dxi_sigma);
            counter=counter+1;
        end
    end
end 
DJ_Main=sum(DJ,3);

J=0.5*sum(sum(Diff.*Diff));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [C, Ceq, Grad_C, Grad_Ceq]=ctr_eigenvalue(x,d,K,EPSILON,Sigma_input_P,Sigma_input_V)
 epsilon=0;
[~, ~, Sigma_P , ~, ~, Sigma_V] = shape_DS(x,d,K); 
for i=1:K
    Sigma_P(1:d,1:d,i)=Sigma_P(1:d,1:d,i)*Sigma_P(1:d,1:d,i)';
    Sigma_V(1:d,1:d,i)=Sigma_V(1:d,1:d,i)*Sigma_V(1:d,1:d,i)';
end
%%%%%%%%%%%%%%%%%%%%%   Constrain on A
% C=zeros(1,2*K*d+2*K*d);
Ceq=zeros(1,K*d*(d-1)/2);
counter=1; 
gain=0.1;
for pointer_of_K=1:K
    for pointer_of_colum_and_row=1:d
        C(counter)=((-1)^(pointer_of_colum_and_row+1))*Comstrain_on_Ak(Sigma_P,d,K,pointer_of_colum_and_row,pointer_of_K)+epsilon;
        counter=counter+1;
    end
end 
% for pointer_of_K=1:K
%     for pointer_of_colum_and_row=1:d
%         C(counter)=gain*(-Comstrain_on_det_Input(Sigma_P,d,K,pointer_of_colum_and_row,pointer_of_K)+epsilon);
%         counter=counter+1;
%     end
% end
for pointer_of_K=1:K
    for pointer_of_colum_and_row=1:d
        C(counter)=(((-1)^(pointer_of_colum_and_row+1))*Comstrain_on_Ak(Sigma_V,d,K,pointer_of_colum_and_row,pointer_of_K)+epsilon);
        counter=counter+1;
    end
end
% for pointer_of_K=1:K
%     for pointer_of_colum_and_row=1:d
%         C(counter)=gain*(-Comstrain_on_det_Input(Sigma_V,d,K,pointer_of_colum_and_row,pointer_of_K)+epsilon);
%         counter=counter+1;
%     end
% end

counter=1;
for pointer_of_K=1:K
    for pointer_of_colum=1:d
        for pointer_of_row=pointer_of_colum:d
            if pointer_of_colum~=pointer_of_row
                Ceq(counter)=epsilon*Comstrain_on_Symetric_A_Position(Sigma_P,d,K,pointer_of_colum,pointer_of_row,pointer_of_K);
                counter=counter+1;
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Derivatives
Grad_Ceq_pi=zeros(K*d,K);
Grad_C_mu=zeros(K*d,d*K);
Grad_C_sigma=zeros(d*K,d*(3*d+1)*K/2);
Grad_C_Sigma_det=zeros(K,d*(3*d+1)*K/2);
Grad_C_handle=zeros(1,d*(3*d+1)*K/2);
counter_main=1;
for pointer_of_K=1:K  
    for pointer_of_colum_and_row=1:d
        counter=1; 
        for j=1:K 
             for colum=1:d
                 for row=colum:d
                    Grad_C_handle(:,counter)=((-1)^(pointer_of_colum_and_row+1))*Diff_constrains_ak_respect_to_sigmainput(Sigma_P,d,K,row,colum,j,pointer_of_colum_and_row,pointer_of_K);
                    counter=counter+1;
                 end
                 for row=1:d
                    Grad_C_handle(:,counter)=((-1)^(pointer_of_colum_and_row+1))*Diff_constrains_ak_respect_to_sigmainputoutput(Sigma_P,d,K,row,colum,j,pointer_of_colum_and_row,pointer_of_K);
                    counter=counter+1;
                 end 
             end 
        end
        Grad_C_sigma(counter_main,:)=Grad_C_handle(1,:);
        counter_main=counter_main+1;
    end
end
counter_main=1;
% for pointer_of_K=1:K 
%     for pointer_of_colum_and_row=1:d
%         Grad_C_Sigma_det(counter_main,:)=gain*Diff_constrains_det_Input_respect_to_sigmainput(Sigma_P,d,K,pointer_of_colum_and_row,pointer_of_K);
%         counter_main=counter_main+1;
%     end
% end 
 Diff_wrt_Pi_and_Mu=zeros(K*d,K+d*K);
 Grad_C_Position=([Grad_Ceq_pi Grad_C_mu Grad_C_sigma]);
 %%% Velocity
Grad_Ceq_pi=zeros(K*d,K);
Grad_C_sigma=zeros(d*K,d*(3*d+1)*K/2);
Grad_C_Sigma_det=zeros(K,d*(3*d+1)*K/2);
Grad_C_handle=zeros(1,d*(3*d+1)*K/2);
counter_main=1;
for pointer_of_K=1:K  
    for pointer_of_colum_and_row=1:d
        counter=1; 
        for j=1:K 
             for colum=1:d
                 for row=colum:d
                    Grad_C_handle(:,counter)=((-1)^(pointer_of_colum_and_row+1))*Diff_constrains_ak_respect_to_sigmainput(Sigma_V,d,K,row,colum,j,pointer_of_colum_and_row,pointer_of_K);
                    counter=counter+1;
                 end
                 for row=1:d
                    Grad_C_handle(:,counter)=((-1)^(pointer_of_colum_and_row+1))*Diff_constrains_ak_respect_to_sigmainputoutput(Sigma_V,d,K,row,colum,j,pointer_of_colum_and_row,pointer_of_K);
                    counter=counter+1;
                 end 
             end 
        end
        Grad_C_sigma(counter_main,:)=Grad_C_handle(1,:);
        counter_main=counter_main+1;
    end
end
% counter_main=1;
% for pointer_of_K=1:K 
%     for pointer_of_colum_and_row=1:d
%         Grad_C_Sigma_det(counter_main,:)=gain*Diff_constrains_det_Input_respect_to_sigmainput(Sigma_V,d,K,pointer_of_colum_and_row,pointer_of_K);
%         counter_main=counter_main+1;
%     end
% end 
 Diff_wrt_Pi_and_Mu=zeros(K*d,K+d*K);
 
 Grad_C_Velocity=([Grad_Ceq_pi Grad_C_mu Grad_C_sigma]);
 Zero=zeros(K*d,(K+d*K+d*(3*d+1)*K/2));
  Grad_C=transpose([Grad_C_Position Zero; Zero Grad_C_Velocity]);
%%%%%%%%%%%%%%%%%%%%%%%% Derivatives of Ceq
 Grad_Ceq_mu=zeros(K*d*(d-1)/2,d*K);
Grad_Ceq_pi=zeros(K*d*(d-1)/2,K);
Grad_Ce_handle=zeros(1,d*(3*d+1)/2);
Grad_Ce_Ak=zeros(K*d*(d-1)/2,d*(3*d+1)*K/2);
counter_main=1;

for pointer_of_K=1:K  
    for Pointer_of_d_colume_main=1:d
        for Pointer_of_d_row_Main=Pointer_of_d_colume_main:d
            if Pointer_of_d_colume_main~=Pointer_of_d_row_Main
                counter=1;
                 for j=1:K
                    for colum=1:d
                        for row=colum:d
                            Grad_Ce_handle(:,counter)=epsilon*Diff_constrains_ak_SYMMETRIC_respect_to_sigmainput(Sigma_P,d,K,row,colum,j,pointer_of_K,Pointer_of_d_row_Main,Pointer_of_d_colume_main);
                            counter=counter+1;
                        end
                        for row=1:d
                            Grad_Ce_handle(:,counter)=epsilon*Diff_constrains_ak_SYMMETRIC_respect_to_sigmainputoutput(Sigma_P,d,K,row,colum,j,pointer_of_K,Pointer_of_d_row_Main,Pointer_of_d_colume_main);
                            counter=counter+1;
                        end 
                    end
                 end
                Grad_Ce_Ak(counter_main,:)=Grad_Ce_handle(1,:);
                counter_main=counter_main+1;
            end
        end
    end
end 
 Zero=zeros(K*d*(d-1)/2,(K+d*K+d*(3*d+1)*K/2));
Grad_Ceq=transpose([Grad_Ceq_pi Grad_Ceq_mu Grad_Ce_Ak Zero]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V] = shape_DS(p,d,K)
% transforming the column of parameters into Priors, Mu, and Sigma
Mu_0_P = zeros(2*d,K);
Sigma_0_P = zeros(2*d,2*d,K);
% Sigma_0_P_handle= zeros(d,2*d,K);
Mu_0_V = zeros(2*d,K);
Sigma_0_V = zeros(2*d,2*d,K);
Priors_0_P=zeros(1,K);
Priors_0_V=zeros(1,K);
hadle=0;
for i=(hadle+1):(K+hadle)
 Priors_0_P(1,i) = p(1,i);
end
hadle=i;
for k=1:K
Mu_0_P(1:d,k) = reshape(p(hadle+1:hadle+d),d,1);
hadle=hadle+d;
end

for k=1:K
    for j=1:d
        for i=j:2*d
          Sigma_0_P(i,j,k)= p(hadle+1);
          Sigma_0_P(j,i,k)= p(hadle+1);
          hadle=hadle+1;
        end
    end
end 
for i=(hadle+1):(K+hadle)
 Priors_0_V(1,i-hadle) = p(1,i);
end
hadle=i;
for k=1:K
Mu_0_V(1:d,k) = reshape(p(hadle+1:hadle+d),d,1);
hadle=hadle+d;
end

for k=1:K
    for j=1:d
        for i=j:2*d
          Sigma_0_V(i,j,k)= p(hadle+1);
          Sigma_0_V(j,i,k)= p(hadle+1);
          hadle=hadle+1;
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function p0 = GMM_2_Parameters(Priors_0_P,Mu_0_P,Sigma_0_P,Priors_0_V, Mu_0_V, Sigma_0_V,d,K)
x1=zeros(1,(K+d*K+d*(d)*K));
x2=zeros(1,(K+d*K+d*(d)*K));

for i=1:(K)
    x1(i)=Priors_0_P(1,i);
end
hadle=i+1;
for k=1:K
x1(hadle:hadle+d-1) = reshape(Mu_0_P(1:d,k),1,d);
hadle=hadle+d;
end
for k=1:K
    for j=1:d
        for i=j:2*d
             x1(hadle) = Sigma_0_P(i,j,k);
             hadle=hadle+1;
        end
    end
end

for i=1:(K)
    x2(i)=Priors_0_V(1,i);
end
hadle=i+1;
for k=1:K
x2(hadle:hadle+d-1) = reshape(Mu_0_V(1:d,k),1,d);
hadle=hadle+d;
end

for k=1:K
    for j=1:d
        for i=j:2*d
             x2(hadle) = Sigma_0_V(i,j,k);
             hadle=hadle+1;
        end
    end
end
p0=[x1 x2]; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_respect_to_pi(Priors, Mu, Sigma,xi_r,d,K,Pointer_of_K,Denominator,Numerator_Input,A)
Output=0;
 Input=xi_r;

Double_Denominator=Denominator.^2+realmin; 
for o=1:K
    if o==Pointer_of_K
        Numerator=Denominator.*Numerator_Input(:,o)-Priors(o,1).*Numerator_Input(:,o).*Numerator_Input(:,o);
        Output=Output+repmat((Numerator./Double_Denominator),1,d)'.*(A(:,:,o)*(Input));
    else
        Numerator=-Priors(o,1).*Numerator_Input(:,o).*Numerator_Input(:,Pointer_of_K);
        Output=Output+repmat((Numerator./Double_Denominator),1,d)'.*(A(:,:,o)*(Input));
    end
end
Diff=Output;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_respect_to_mu_input(Priors, Mu, Sigma,xi_r,d,K,Pointer_of_d,Pointer_of_K,Denominator,Numerator_Input,A)
Output=0;
 Input=xi_r;
 nbData=size(xi_r,2);
 Diff_P_handle=zeros(d,nbData);
Diff_P_handle(Pointer_of_d,1:nbData)=1;

Double_Denominator=Denominator.^2+realmin;

Sigmainput=Sigma(1:d,1:d,Pointer_of_K);
Muinput=Mu(1:d,Pointer_of_K);
Diff_P_Pointer_of_K=-(0.5*(2*diag(transpose(Diff_P_handle)/(Sigmainput)*(repmat(Muinput,1,nbData)-Input))).*Numerator_Input(:,Pointer_of_K));
for o=1:K
    if o==Pointer_of_K
        Output=Output+repmat(((Priors(o,1)*(Denominator-Numerator_Input(:,o)*Priors(o,1)).*Diff_P_Pointer_of_K)./Double_Denominator),1,d)'.*(A(:,:,o)*(Input));
    else
        Output=Output+repmat((-Priors(o,1)*Priors(Pointer_of_K,1)*Numerator_Input(:,o).*Diff_P_Pointer_of_K./Double_Denominator),1,d)'.*(A(:,:,o)*(Input));
    end
end    

Diff=Output;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_respect_to_sigmainput(Priors, Mu, Sigma,xi_r,d,K,Pointer_of_d_row,Pointer_of_d_colum,Pointer_of_K,Denominator,Numerator_P,A,Sigma_input)
 nbData=size(xi_r,2);
Diff_P_handle=zeros(d,d);
Diff_P_handle(Pointer_of_d_row,Pointer_of_d_colum)=1;
Diff_P_handle(Pointer_of_d_colum,Pointer_of_d_row)=1;
Diff_P_handle=Diff_P_handle*Sigma_input+Sigma_input*Diff_P_handle;
% Diff_P_handle=repmat(Diff_P_handle,1,nbData);
Output=0;
 Input=xi_r;
 
Double_Denominator=Denominator.^2+realmin;

o=Pointer_of_K;
Sigmainput=Sigma(1:d,1:d,o); 
SigmaInput_Output=Sigma(d+1:2*d,1:d,o);
Muinput=Mu(1:d,o);


% exp(-0.5*abs(prob)) / sqrt((2*pi)^nbVar * (abs(det(Sigma))+realmin))

% Diff_P_Pointer_of_K;


% Input_A = Input - repmat(Muinput,1,nbData);
% Diffprob = sign(sum((Input_A'/Sigmainput).*Input_A', 2)).*sum((Input_A'*(-inv(Sigmainput)*Diff_P_handle*inv(Sigmainput))).*Input_A', 2);
% DiffSigma=-0.5*(abs(det(Sigmainput)))^(-1)*sign(det(Sigmainput))*det(Sigmainput)*trace(inv(Sigmainput)*Diff_P_handle);
% Diff_P_Pointer_of_K=-0.5*Diffprob.*Numerator_P(:,o)+Numerator_P(:,o)*DiffSigma;

Input_A = Input - repmat(Muinput,1,nbData);
Diffprob = sum((Input_A'*(-Sigmainput\Diff_P_handle/Sigmainput)).*Input_A', 2);
DiffSigma=-0.5*((det(Sigmainput)))^(-1)*det(Sigmainput)*trace(Sigmainput\Diff_P_handle);
Diff_P_Pointer_of_K=-0.5*Diffprob.*Numerator_P(:,o)+Numerator_P(:,o)*DiffSigma;



% Sign=sign(diag((transpose(Input-repmat(Muinput,1,nbData))/(Sigmainput))*(Input-repmat(Muinput,1,nbData)))).*(Numerator_P(:,o)./repmat(Priors(o,1)',nbData,1));
% Handle=(1/sqrt(((2*pi)^(K+d))))*(-0.5*(abs(det(Sigmainput)))^(-3/2))*sign(det(Sigmainput))*(det(Sigmainput)*trace((Sigmainput)\Diff_P_handle));
% Diff_P_Pointer_of_K=A.*diag(-0.5*(transpose(repmat(Muinput,1,nbData)-Input)*(-inv(Sigmainput)*Diff_P_handle/Sigmainput))*(repmat(Muinput,1,nbData)-Input)).*Sign...
% +Handle*exp(-0.5*diag(abs((transpose(repmat(Muinput,1,nbData)-Input)/(Sigmainput))*(repmat(Muinput,1,nbData)-Input))));

for o=1:K
    if o==Pointer_of_K
        PI=Priors(o,1);
        Sigmainput=Sigma(1:d,1:d,o); 
        SigmaInput_Output=Sigma(d+1:2*d,1:d,o);
        Numerator=PI*(Denominator-Numerator_P(:,o)*PI).*Diff_P_Pointer_of_K;  
        Output=Output+repmat((Numerator./Double_Denominator),1,d)'.*(A(:,:,o)*(Input));
        Output=Output+repmat((PI*Numerator_P(:,o)./Denominator),1,d)'.*(SigmaInput_Output*(-Sigmainput\Diff_P_handle/Sigmainput)*(Input));
    else
        PI_Pointer_of_K=Priors(Pointer_of_K,1);
        PI=Priors(o,1); 
        Numerator=-PI*PI_Pointer_of_K.*Numerator_P(:,o).*Diff_P_Pointer_of_K;
        Output=Output+repmat(Numerator./Double_Denominator,1,d)'.*(A(:,:,o)*Input);
% Output=Output+(A(:,:,o)*(Input));
    end
end  

Diff=Output; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_respect_to_sigmainputoutput(Priors, Mu, Sigma,xi_r,d,K,Pointer_of_d_row,Pointer_of_d_colum,Pointer_of_K,Denominator,Numerator_Input)
Diff_P_handle=zeros(d,d);
Diff_P_handle(Pointer_of_d_row,Pointer_of_d_colum)=1;
Diff=0;
 Input=xi_r;
Diff=Diff+repmat((Priors(Pointer_of_K,1)*Numerator_Input(:,Pointer_of_K)./Denominator),1,d)'.*((Diff_P_handle/(Sigma(1:d,1:d,Pointer_of_K)))*(Input));
% Diff=Diff+((Diff_P_handle/(Sigma(1:d,1:d,Pointer_of_K)))*(Input));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Constrain on A_k_Eigenvaue
function C=Comstrain_on_Ak(Sigma,d,~,pointe_of_colum_and_row,pointe_of_K)
output=Sigma((d+1):2*d,1:d,pointe_of_K)/Sigma(1:d,1:d,pointe_of_K);
C=det(output(1:pointe_of_colum_and_row,1:pointe_of_colum_and_row)+transpose(output(1:pointe_of_colum_and_row,1:pointe_of_colum_and_row)));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Constrain on Det_Sigma_input
function C=Comstrain_on_det_Input(Sigma,~,~,pointe_of_colum_and_row,pointer_of_K)
% epsilon=0.00000001;
Sigmainput=Sigma(1:pointe_of_colum_and_row,1:pointe_of_colum_and_row,pointer_of_K);
C=det(Sigmainput+transpose(Sigmainput));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Constrain on A_position_Symmetric
function Ceq=Comstrain_on_Symetric_A_Position(Sigma,d,~,pointer_of_colum,pointer_of_row,pointe_of_K)
A=Sigma((d+1):2*d,1:d,pointe_of_K)/Sigma(1:d,1:d,pointe_of_K);
Ceq=A(pointer_of_row,pointer_of_colum)-A(pointer_of_colum,pointer_of_row);
%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_constrains_ak_respect_to_sigmainputoutput(Sigma,d,~,Pointer_of_d_row,Pointer_of_d_colum,Pointer_of_K,pointer_of_colum_and_row,Pointer_of_KK)
Diff_P_handle=zeros(d,d);
Diff_P_handle(Pointer_of_d_row,Pointer_of_d_colum)=1;
output=0;
    if Pointer_of_KK==Pointer_of_K
        handle=Sigma((d+1):2*d,1:d,Pointer_of_K)/(Sigma(1:d,1:d,Pointer_of_K));
        handle=handle+transpose(handle);
        handle2=handle(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row);
        handle3=Diff_P_handle/Sigma(1:d,1:d,Pointer_of_K)+transpose(Diff_P_handle/Sigma(1:d,1:d,Pointer_of_K));
        output=det(handle2)*trace(handle2\(handle3(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row)));
    end
Diff=output;
%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_constrains_ak_respect_to_sigmainput(Sigma,d,~,Pointer_of_d_row,Pointer_of_d_colum,Pointer_of_K,pointer_of_colum_and_row,Pointer_of_KK)
Diff_P_handle=zeros(d,d);
Diff_P_handle(Pointer_of_d_row,Pointer_of_d_colum)=1;
Diff_P_handle(Pointer_of_d_colum,Pointer_of_d_row)=1;
Diff_P_handle=Diff_P_handle*sqrtm(Sigma(1:d,1:d,Pointer_of_K))+sqrtm(Sigma(1:d,1:d,Pointer_of_K))*Diff_P_handle;
output=0;
    if Pointer_of_KK==Pointer_of_K
        Sigmainput=Sigma(1:d,1:d,Pointer_of_K);
        SigmaInput_Output=Sigma((d+1):2*d,1:d,Pointer_of_K);
        handle=SigmaInput_Output*(inv(Sigmainput));
        handle=handle+transpose(handle);
        handle2=handle(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row);
        handle3=SigmaInput_Output*(-inv(Sigmainput)*Diff_P_handle/Sigmainput)+transpose(SigmaInput_Output*(-inv(Sigmainput)*Diff_P_handle/Sigmainput));
        output=det(handle2)*trace(handle2\(handle3(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row)));
    end
Diff=output;
%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_constrains_ak_SYMMETRIC_respect_to_sigmainput(Sigma,d,~,Pointer_of_d_row,Pointer_of_d_colum,Pointer_of_K,Pointer_of_KK,Pointer_of_d_row_Main,Pointer_of_d_colume_main)
Diff_P_handle=zeros(d,d);
Diff_P_handle(Pointer_of_d_row,Pointer_of_d_colum)=1;
Diff_P_handle(Pointer_of_d_colum,Pointer_of_d_row)=1;
Diff_P_handle=Diff_P_handle*sqrtm(Sigma(1:d,1:d,Pointer_of_K))+sqrtm(Sigma(1:d,1:d,Pointer_of_K))*Diff_P_handle;
handle2=0;
if Pointer_of_KK==Pointer_of_K
    Sigmainput=Sigma(1:d,1:d,Pointer_of_K);
    SigmaInput_Output=Sigma((d+1):2*d,1:d,Pointer_of_K);
    handle=SigmaInput_Output*(-inv(Sigmainput)*Diff_P_handle/Sigmainput);
    handle2=handle(Pointer_of_d_row_Main,Pointer_of_d_colume_main)-handle(Pointer_of_d_colume_main,Pointer_of_d_row_Main);
end
Diff=handle2;
%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_constrains_det_Input_respect_to_sigmainput(Sigma,d,K,pointer_of_colum_and_row,pointer_of_K)

Diff_wrt_K=zeros(1,d*(3*d+1)/2);
Sigmainput=Sigma(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row,pointer_of_K);
counter=1;
Diff=[];
Diff_Det=zeros(1,d*(3*d+1)/2);
for colum=1:d
    for row=colum:d
        if (colum<=pointer_of_colum_and_row)&&(row<=pointer_of_colum_and_row)
           handle=zeros(pointer_of_colum_and_row,pointer_of_colum_and_row);
           handle(row,colum)=1;
           handle(colum,row)=1;
           Diff_Det(1,counter)=(-det(Sigmainput+transpose(Sigmainput))*trace((Sigmainput+transpose(Sigmainput))\(handle+handle)));
           counter=counter+1;
        else
            Diff_Det(1,counter)=0;
            counter=counter+1;
        end
    end
    for row=1:d
        Diff_Det(1,counter)=0;
        counter=counter+1;
    end
end
 
handle_Diff=Diff_Det;
for i=1:K
    if i==pointer_of_K
        Diff=[Diff handle_Diff];
    else
        Diff=[Diff Diff_wrt_K];
    end
end
%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_constrains_ak_SYMMETRIC_respect_to_sigmainputoutput(Sigma,d,~,Pointer_of_d_row,Pointer_of_d_colum,Pointer_of_K,Pointer_of_KK,Pointer_of_d_row_Main,Pointer_of_d_colume_main)
Diff_P_handle=zeros(d,d);
Diff_P_handle(Pointer_of_d_row,Pointer_of_d_colum)=1;
handle2=0;
    if Pointer_of_KK==Pointer_of_K
        handle=Diff_P_handle/(Sigma(1:d,1:d,Pointer_of_K));
        handle2=handle(Pointer_of_d_row_Main,Pointer_of_d_colume_main)-handle(Pointer_of_d_colume_main,Pointer_of_d_row_Main);
    end
Diff=handle2;
