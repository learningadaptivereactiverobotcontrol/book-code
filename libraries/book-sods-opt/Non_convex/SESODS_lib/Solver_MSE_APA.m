function [Priors_P, Mu_P, Sigma_P ,Mu_V ,Sigma_V,P,popt]=Solver_MSE_APA(Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V,P,Data_A,EPSILON,options)
d = size(Sigma_0_P,1)/2; %dimension of the model
K = size(Sigma_0_P,3);
if options.display
    str = 'iter';
else
    str = 'off';
end
optNLP = optimset('Algorithm', 'interior-point','Diagnostics','on', 'LargeScale', 'on',...
    'GradObj', 'on', 'GradConstr', 'on', ...
    'Display', 'iter', 'TolX',options.tol_stopping, 'TolFun', options.tol_stopping, 'TolCon', 1e-8, ...
    'MaxFunEval', 20000, 'MaxIter', options.max_iter, 'DiffMinChange', ...
    options.tol_stopping, 'Hessian','bfgs','display',str,'AlwaysHonorConstraints','bounds','FinDiffRelStep',realmin,...
    'InitTrustRegionRadius',10000000000000000,'ScaleProblem','obj-and-constr','SubproblemAlgorithm','cg'); %,'FinDiffType','central''DerivativeCheck','on' interior-point


for i=1:K
    Sigma_0_P(1:d,1:d,i)=sqrtm(Sigma_0_P(1:d,1:d,i));
    Sigma_0_V(1:d,1:d,i)=sqrtm(Sigma_0_V(1:d,1:d,i));
end
Sigma_input_P=Sigma_0_P(1:d,1:d,:);
Sigma_input_V=Sigma_0_V(1:d,1:d,:);
p0 = GMM_2_Parameters(Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V,P,d,K);

[A,b,Aeq,beq,lb,ub,Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V]=Initilazation(p0,d,K);
p0 = GMM_2_Parameters(Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V,P,d,K);


obj_handle = @(p)obj((p),Data_A,d,K,Sigma_input_P,Sigma_input_V,P);
ctr_handle = @(p) ctr_eigenvalue((p),d,K,EPSILON,Sigma_input_P,Sigma_input_V,P);
%  ctr_handle=[];

[popt,fval,exitflag] = fmincon(obj_handle, p0,A,b,[],[],lb,ub,ctr_handle,optNLP);
% disp('The value of the objective function is:')
[C, ~, ~, ~]=ctr_eigenvalue(popt,d,K,EPSILON,Sigma_input_P,Sigma_input_V)


%  optNLP = optimset('Algorithm', 'sqp','Diagnostics','on', 'LargeScale', 'on',...
%         'GradObj', 'on', 'GradConstr', 'on','FinDiffType','central', ...
%         'Display', 'iter', 'TolX',options.tol_stopping/100, 'TolFun', options.tol_stopping, 'TolCon', options.TolCon, ...
%         'MaxFunEval', 20000, 'MaxIter', options.max_iter, 'DiffMinChange', ...
%          options.tol_stopping, 'Hessian','bfgs','display',str,'AlwaysHonorConstraints','bounds','UseParallel','always');
% [popt,fval,exitflag] = fmincon(obj_handle, popt,A,b,Aeq,beq,lb,ub,ctr_handle,optNLP);

disp('The value of the objective function is:')
disp(fval)
Massage(exitflag);
[Priors_P, Mu_P, Sigma_P ,Mu_V ,Sigma_V,P] = shape_DS(popt,d,K);
for i=1:K
    Sigma_P(1:d,1:d,i)=Sigma_P(1:d,1:d,i)*Sigma_P(1:d,1:d,i)';
    Sigma_V(1:d,1:d,i)=Sigma_V(1:d,1:d,i)*Sigma_V(1:d,1:d,i)';
end
% Sigma_P(1:d,1:d,:)=Sigma_input_P;
% Sigma_V(1:d,1:d,:)=Sigma_input_V;
p0 = GMM_2_Parameters(Priors_P,Mu_P,Sigma_P, Mu_V, Sigma_V,P,d,K);
[Priors_P, Mu_P, Sigma_P, Mu_V ,Sigma_V]=Stabilizing_B_K(Priors_P,Mu_P,Sigma_P,Mu_V,Sigma_V,d,K);
Priors_P=abs(Priors_P)/sum(Priors_P);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [A,b,Aeq,beq,lb,ub,Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V]=Initilazation(p0,d,K)
Aeq=[];
beq=[];
% A=zeros(2*(K+2*d*K+4*d*d*K),2);
Size_all=size(p0);
A=zeros(Size_all(1,2),2*K);
lb=[];
ub=[];
Aeq=zeros(2,Size_all(1,2));



[Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V] = shape_DS(p0,d,K);
% Shift=K+d*K+d*(3*d+1)*K/2;
% Shift=K+d*K+d*(3*d+1)*K/2;



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
% for i=1:K
%     A(i+Shift,1)=1;
% end
%
% for i=1:K
%     A(i+Shift,2)=-1;
% end

A=transpose(A);

for i=1:K
    Aeq(1,i)=1;
end
%  for i=1:K
%     Aeq(2,i+Shift)=1;
%  end
beq(1)=0;
beq(2)=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Priors_P, Mu_P, Sigma_P, Mu_V, Sigma_V]=Stabilizing_B_K(Priors_P,Mu_P,Sigma_P,Mu_V,Sigma_V,d,K)
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
function [J, DJ_Main]=obj(p,Data,d,K,Sigma_input_P,Sigma_input_V,P)

% This function computes the derivative of the likelihood objective function
% w.r.t. optimization parameters.
DJ_Main=0;
nData = size(Data,2);
DJ=zeros(1,size(p,2),nData);
DDX=zeros(d,nData);
Diff=zeros(d,nData);

[Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V,~] = shape_DS(p,d,K);

for i=1:K
    Sigma_0_P(1:d,1:d,i)=Sigma_0_P(1:d,1:d,i)*Sigma_0_P(1:d,1:d,i)';
    Sigma_0_V(1:d,1:d,i)=Sigma_0_V(1:d,1:d,i)*Sigma_0_V(1:d,1:d,i)';
end

i=1;

X  = Data(1:d,:);
DX = Data(d+1:2*d,:);
[DDX,Denominator,Numerator_P,Numerator_V,A_P,A_V]= SE_DS(Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V,DX,X);
Diff=(DDX-Data(2*d+1:3*d,:));
%%% Derivatives
%%% Position
DJ_DO=transpose(Diff);
counter=1;
for j=1:K
    Dxi_PI=Diff_respect_to_pi(transpose(Priors_0_P),X,DX,d,K,j,Denominator,Numerator_P,Numerator_V,A_P,A_V);
    DJ(:,counter,:)=diag(DJ_DO*Dxi_PI);
    counter=counter+1;
end
for j=1:K
    for o=1:d
        Dxi_mu=Diff_respect_to_mu_input(transpose(Priors_0_P), Mu_0_P, Sigma_0_P,X,DX,d,K,o,j,Denominator,Numerator_P,Numerator_V,A_P,A_V);
        DJ(:,counter,:)=diag(DJ_DO*Dxi_mu);
        counter=counter+1;
    end
end
for j=1:K
    for colum=1:d
        for row=colum:d
            Dxi_sigma=Diff_respect_to_sigmainput(transpose(Priors_0_P), Mu_0_P, Sigma_0_P,X,DX,d,K,row,colum,j,Denominator,Numerator_P,Numerator_V,A_P,A_V,Sigma_input_P(:,:,j));
            DJ(:,counter,1:nData)=diag(DJ_DO*Dxi_sigma);%;
            counter=counter+1;
        end
        for row=1:d
            Dxi_sigma=Diff_respect_to_sigmainputoutput(transpose(Priors_0_P), Sigma_0_P,X,d,row,colum,j,Denominator,Numerator_P,Numerator_V);
            DJ(:,counter,:)=diag(DJ_DO*Dxi_sigma);
            counter=counter+1;
        end
    end
end
%
% %% Velocity
for j=1:K
    for o=1:d
        Dxi_mu=Diff_respect_to_mu_input(transpose(Priors_0_P), Mu_0_V, Sigma_0_V,DX,X,d,K,o,j,Denominator,Numerator_V,Numerator_P,A_V,A_P);
        DJ(:,counter,:)=diag(DJ_DO*Dxi_mu);
        counter=counter+1;
    end
end
for j=1:K
    for colum=1:d
        for row=colum:d
            Dxi_sigma=Diff_respect_to_sigmainput(transpose(Priors_0_P), Mu_0_V, Sigma_0_V,DX,X,d,K,row,colum,j,Denominator,Numerator_V,Numerator_P,A_V,A_P,Sigma_input_V(:,:,j));
            DJ(:,counter,1:nData)=diag(DJ_DO*Dxi_sigma);%diag(DJ_DO*Dxi_sigma);
            counter=counter+1;
        end
        for row=1:d
            Dxi_sigma=Diff_respect_to_sigmainputoutput(transpose(Priors_0_P), Sigma_0_V,DX,d,row,colum,j,Denominator,Numerator_V,Numerator_P);
            DJ(:,counter,:)=diag(DJ_DO*Dxi_sigma);
            counter=counter+1;
        end
    end
end
for colum=1:2*d
    for row=colum:2*d
        DJ(:,counter,1:nData)=0;
        counter=counter+1;
    end
end
DJ_Main=sum(DJ,3);

J=0.5*sum(sum(Diff.*Diff));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Dx_main,Denominator,Numerator_P,Numerator_V,A_P,A_V]= SE_DS(Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V,DX,X)

K=size(Sigma_0_P);
if max(size(K))==2
    K(1,3)=1;
end
d=size(X);
nData = size(X,2);
Input=X;
DInput=DX;
A_V=zeros(d(1,1),d(1,1),K(1,3));
A_P=zeros(d(1,1),d(1,1),K(1,3));
Numerator_P=zeros(nData,K(1,3));
Numerator_V=zeros(nData,K(1,3));
%%% Position

Output=0;
Denominator=0;
for j=1:K(1,3)
    Sigmainput_P=Sigma_0_P(1:d(1,1),1:d(1,1),j);
    Muinput_P=Mu_0_P(1:d(1,1),j);
    Sigmainput_V=Sigma_0_V(1:d(1,1),1:d(1,1),j);
    Muinput_V=Mu_0_V(1:d(1,1),j);
    Numerator_P(:,j)=repmat(1/sqrt(((2*pi)^(d(1,1)))*(det(Sigmainput_P))),nData,1).*exp(-0.5.*diag(((transpose(Input-repmat(Muinput_P,1,nData))/(Sigmainput_P))*(Input-repmat(Muinput_P,1,nData)))));
    Numerator_V(:,j)=repmat(1/sqrt(((2*pi)^(d(1,1)))*(det(Sigmainput_V))),nData,1).*exp(-0.5.*diag(((transpose(DInput-repmat(Muinput_V,1,nData))/(Sigmainput_V))*(DInput-repmat(Muinput_V,1,nData)))));
end
Denominator=sum(repmat(Priors_0_P,nData,1).*Numerator_P.*Numerator_V,2);
if nnz(Denominator.^2)<size(Denominator,1)
    Denominator=Denominator+sqrt(realmin);
end

for o=1:K(1,3)
    Sigmainput_P=Sigma_0_P(1:d(1,1),1:d(1,1),o);
    SigmaInput_Output_P=Sigma_0_P(d(1,1)+1:2*d(1,1),1:d(1,1),o);
    Sigmainput_V=Sigma_0_V(1:d(1,1),1:d(1,1),o);
    SigmaInput_Output_V=Sigma_0_V(d(1,1)+1:2*d(1,1),1:d(1,1),o);
    PI=Priors_0_P(1,o);
    A_P(:,:,o)=SigmaInput_Output_P/(Sigmainput_P);
    A_V(:,:,o)=SigmaInput_Output_V/(Sigmainput_V);
    Output=Output+repmat((PI*Numerator_P(:,o).*Numerator_V(:,o)./Denominator),1,d(1,1))'.*(A_P(:,:,o)*(Input)+A_V(:,:,o)*(DInput));
end
% Output=(Sigma_0_P(1:d(1,1),1:d(1,1),1))\(Input);
Dx_main=Output;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [C, Ceq, Grad_C, Grad_Ceq]=ctr_eigenvalue(x,d,K,EPSILON,Sigma_input_P,Sigma_input_V,P)
epsilon=EPSILON-eps;
penalty=100000000;
% penalty=100000000;
[~, ~, Sigma_P , ~, Sigma_V,P] = shape_DS(x,d,K);
% Sigma_P(1:d,1:d,:)=Sigma_input_P;
% Sigma_V(1:d,1:d,:)=Sigma_input_V;
for i=1:K
    Sigma_P(1:d,1:d,i)=Sigma_P(1:d,1:d,i)*Sigma_P(1:d,1:d,i)';
    Sigma_V(1:d,1:d,i)=Sigma_V(1:d,1:d,i)*Sigma_V(1:d,1:d,i)';
end
Ceq=[];
Grad_Ceq=[];
%%%%%%%%%%%%%%%%%%%%%   Constrain on A
% C=zeros(1,2*K*d+2*K*d);
C=zeros(1,2*K*d+2*d);
counter=1;
for pointer_of_K=1:K
    A=[zeros(d,d) eye(d,d);Sigma_P(d+1:2*d,1:d,pointer_of_K)/(Sigma_P(1:d,1:d,pointer_of_K)) Sigma_V(d+1:2*d,1:d,pointer_of_K)/(Sigma_V(1:d,1:d,pointer_of_K))];
    for pointer_of_colum_and_row=1:2*d
        C(counter)=penalty*((-1)^(pointer_of_colum_and_row+1))*Comstrain_on_Ak(transpose(A)*P+P*A,pointer_of_colum_and_row)+penalty*epsilon;
        counter=counter+1;
    end
end
for pointer_of_colum_and_row=1:2*d
    C(counter)=-penalty*Comstrain_on_Ak(P,pointer_of_colum_and_row)+penalty*epsilon;
    counter=counter+1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Derivatives
Grad_Ceq_pi=zeros(2*K*d,K);
Grad_C_mu=zeros(2*K*d,d*K);
Grad_C_sigma=zeros(2*d*K,d*(3*d+1)*K/2);
Grad_C_handle=zeros(1,d*(d)*K);
counter_main=1;
for pointer_of_K=1:K
    for pointer_of_colum_and_row=1:2*d
        counter=1;
        for j=1:K
            for colum=1:d
                for row=colum:d
                    Grad_C_handle(:,counter)=((-1)^(pointer_of_colum_and_row+1))*Diff_constrains_ak_respect_to_sigmainput(Sigma_P,Sigma_V,P,d,K,row,colum,j,pointer_of_colum_and_row,pointer_of_K);
                    %                       Grad_C_handle(:,counter)=0;
                    counter=counter+1;
                end
                for row=1:d
                    Grad_C_handle(:,counter)=((-1)^(pointer_of_colum_and_row+1))*Diff_constrains_ak_respect_to_sigmainputoutput(Sigma_P,Sigma_V,P,d,row,colum,j,pointer_of_colum_and_row,pointer_of_K);
                    counter=counter+1;
                end
            end
        end
        Grad_C_sigma(counter_main,:)=Grad_C_handle(1,:);
        counter_main=counter_main+1;
    end
end
Grad_C_Position=([Grad_Ceq_pi Grad_C_mu Grad_C_sigma]);
%%% Velocity
Grad_C_mu=zeros(2*K*d,d*K);
Grad_C_sigma=zeros(2*d*K,d*(3*d+1)*K/2);
Grad_C_handle=zeros(1,d*(d)*K);
counter_main=1;
for pointer_of_K=1:K
    for pointer_of_colum_and_row=1:2*d
        counter=1;
        for j=1:K
            for colum=1:d
                for row=colum:d
                    Grad_C_handle(:,counter)=((-1)^(pointer_of_colum_and_row+1))*Diff_constrains_ak_respect_to_sigmainput_V(Sigma_P,Sigma_V,P,d,K,row,colum,j,pointer_of_colum_and_row,pointer_of_K);
                    %                       Grad_C_handle(:,counter)=0;
                    counter=counter+1;
                end
                for row=1:d
                    Grad_C_handle(:,counter)=((-1)^(pointer_of_colum_and_row+1))*Diff_constrains_ak_respect_to_sigmainputoutput_V(Sigma_P,Sigma_V,P,d,row,colum,j,pointer_of_colum_and_row,pointer_of_K);
                    counter=counter+1;
                end
            end
        end
        Grad_C_sigma(counter_main,:)=Grad_C_handle(1,:);
        counter_main=counter_main+1;
    end
end
Grad_C_Velocity=([Grad_C_mu Grad_C_sigma]);
Grad_C_P=zeros(2*d*K,d*(2*d+1));
Grad_C_handle=zeros(1,d*(2*d+1));
counter_main=1;
for pointer_of_colum_and_row=1:2*d
    counter=1;
    for colum=1:2*d
        for row=colum:2*d
            Grad_C_handle(:,counter)=((-1)^(pointer_of_colum_and_row+1))*Diff_constrains_ak_respect_to_P(Sigma_P,Sigma_V,P,d,row,colum,1,pointer_of_colum_and_row,1);
            counter=counter+1;
        end
    end
    Grad_C_P(counter_main,:)=Grad_C_handle(1,:);
    counter_main=counter_main+1;
end

Grad_C_P_P=zeros(2*d,d*(2*d+1));
Grad_C_handle=zeros(1,d*(2*d+1));
counter_main=1;
for pointer_of_colum_and_row=1:2*d
    counter=1;
    for colum=1:2*d
        for row=colum:2*d
            Grad_C_handle(:,counter)=-Diff_constrains_P_respect_to_P(P,d,row,colum,1,pointer_of_colum_and_row,1);
            counter=counter+1;
        end
    end
    Grad_C_P_P(counter_main,:)=Grad_C_handle(1,:);
    counter_main=counter_main+1;
end
Grad_C=transpose([Grad_C_Position Grad_C_Velocity Grad_C_P;zeros(2*d,(K+d*K+d*(3*d+1)*K/2)) zeros(2*d,(d*K+d*(3*d+1)*K/2)) Grad_C_P_P]);
Grad_C=Grad_C*penalty;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V,P] = shape_DS(p,d,K)
% transforming the column of parameters into Priors, Mu, and Sigma
Mu_0_P = zeros(2*d,K);
Sigma_0_P = zeros(2*d,2*d,K);
P=zeros(2*d,2*d);
% Sigma_0_P_handle= zeros(d,2*d,K);
Mu_0_V = zeros(2*d,K);
Sigma_0_V = zeros(2*d,2*d,K);
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


% for k=1:K
%     Sigma_0_P(1:2*d,1:2*d,k)= reshape(p(hadle+1:hadle+4*d*d),2*d,2*d);
% hadle=hadle+4*d*d;
% end
% hadle=i;
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
for j=1:2*d
    for i=j:2*d
        P(i,j)= p(hadle+1);
        P(j,i)= p(hadle+1);
        hadle=hadle+1;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function p0 = GMM_2_Parameters(Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V,P,d,K)
x1=zeros(1,(K+d*K+d*(3*d+1)*K/2));
x2=zeros(1,(d*K+d*(3*d+1)*K/2));
x3=zeros(1,(d*(2*d+1)));

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
hadle=1;
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
hadle=1;
for j=1:2*d
    for i=j:2*d
        x3(hadle) = P(i,j);
        hadle=hadle+1;
    end
end
p0=[x1 x2 x3];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_respect_to_pi(Priors,xi_r,Dxi_r,d,K,Pointer_of_K,Denominator,Numerator_P,Numerator_V,A_P,A_V)
Output=0;
Input=xi_r;
DInput=Dxi_r;


Double_Denominator=Denominator.^2+realmin;

for o=1:K
    if o==Pointer_of_K
        Numerator=Denominator.*Numerator_P(:,o).*Numerator_V(:,o)-Priors(o,1)*Numerator_P(:,o).*Numerator_P(:,o).*Numerator_V(:,o).*Numerator_V(:,o);
        Output=Output+repmat(Numerator./Double_Denominator,1,d)'.*(A_P(:,:,o)*(Input)+A_V(:,:,o)*(DInput));
    else
        Numerator=-Priors(o,1)*Numerator_P(:,o).*Numerator_P(:,Pointer_of_K).*Numerator_V(:,o).*Numerator_V(:,Pointer_of_K);
        Output=Output+repmat(Numerator./Double_Denominator,1,d)'.*(A_P(:,:,o)*(Input)+A_V(:,:,o)*(DInput));
    end
end
Diff=Output;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_respect_to_mu_input(Priors, Mu, Sigma,xi_r,Dxi_r,d,K,Pointer_of_d,Pointer_of_K,Denominator,Numerator_P,Numerator_V,A_P,A_V)
Diff_P_handle=zeros(d,1);
Diff_P_handle(Pointer_of_d,1)=1;

Output=0;
Input=xi_r;
DInput=Dxi_r;
nbData=size(xi_r,2);
Diff_P_handle(Pointer_of_d,1:nbData)=1;

Double_Denominator=Denominator.^2+realmin;
o=Pointer_of_K;
Sigmainput=Sigma(1:d,1:d,o);
Muinput=Mu(1:d,o);
Diff_P_Pointer_of_K=-((0.5*(2*diag(transpose(Diff_P_handle)/(Sigmainput)*(repmat(Muinput,1,nbData)-Input)))).*Numerator_P(:,Pointer_of_K));

for o=1:K
    if o==Pointer_of_K
        Output=Output+repmat(((Priors(o,1)*Numerator_V(:,o).*(Denominator-Numerator_V(:,o).*Numerator_P(:,o).*Priors(o,1)).*Diff_P_Pointer_of_K)./Double_Denominator),1,d)'.*(A_P(:,:,o)*(Input)+A_V(:,:,o)*(DInput));
    else
        Output=Output+repmat((-Priors(o,1)*Priors(Pointer_of_K,1)*Numerator_P(:,o).*Numerator_V(:,o).*Numerator_V(:,Pointer_of_K).*Diff_P_Pointer_of_K./Double_Denominator),1,d)'.*(A_P(:,:,o)*(Input)+A_V(:,:,o)*(DInput));
    end
end

Diff=Output;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_respect_to_sigmainput(Priors, Mu, Sigma,xi_r,Dxi_r,d,K,Pointer_of_d_row,Pointer_of_d_colum,Pointer_of_K,Denominator,Numerator_P,Numerator_V,A_P,A_V,Sigma_input)
Diff_P_handle=zeros(d,d);
Diff_P_handle(Pointer_of_d_row,Pointer_of_d_colum)=1;
Diff_P_handle(Pointer_of_d_colum,Pointer_of_d_row)=1;
Diff_P_handle=Diff_P_handle*Sigma_input+Sigma_input*Diff_P_handle;
Output=0;
Input=zeros(d(1,1),1);
DInput=zeros(d(1,1),1);
% Outputcontrol=0;
Input=xi_r;
DInput=Dxi_r;
nbData=size(xi_r,2);

Double_Denominator=Denominator.^2;

PI_Pointer_of_K=Priors(Pointer_of_K,1);
Sigmainput=Sigma(1:d,1:d,Pointer_of_K);
Muinput=Mu(1:d,Pointer_of_K);

Input_A = Input - repmat(Muinput,1,nbData);
Diffprob = sum((Input_A'*(-Sigmainput\Diff_P_handle/Sigmainput)).*Input_A', 2);
DiffSigma=-0.5*((det(Sigmainput)))^(-1)*det(Sigmainput)*trace(Sigmainput\Diff_P_handle);
Diff_P_Pointer_of_K=-0.5*Diffprob.*Numerator_P(:,Pointer_of_K)+Numerator_P(:,Pointer_of_K)*DiffSigma;
for o=1:K
    if o==Pointer_of_K
        Sigmainput=Sigma(1:d,1:d,o);
        SigmaInput_Output=Sigma(d+1:2*d,1:d,o);
        Numerator=PI_Pointer_of_K*Numerator_V(:,Pointer_of_K).*(Denominator-PI_Pointer_of_K*Numerator_P(:,Pointer_of_K).*Numerator_V(:,Pointer_of_K)).*Diff_P_Pointer_of_K;
        Output=Output+repmat((Numerator./Double_Denominator),1,d)'.*(A_P(:,:,Pointer_of_K)*(Input)+A_V(:,:,Pointer_of_K)*(DInput));
        Output=Output+repmat((PI_Pointer_of_K*Numerator_P(:,Pointer_of_K).*Numerator_V(:,Pointer_of_K)./Denominator),1,d)'.*(SigmaInput_Output*(-Sigmainput\Diff_P_handle/Sigmainput)*(Input));
    else
        PI=Priors(o,1);
        Numerator=-PI*PI_Pointer_of_K*Numerator_P(:,o).*Numerator_V(:,o).*Diff_P_Pointer_of_K.*Numerator_V(:,Pointer_of_K);
        Output=Output+repmat((Numerator./Double_Denominator),1,d)'.*(A_P(:,:,Pointer_of_K)*(Input)+A_V(:,:,Pointer_of_K)*(DInput));
        
    end
end

Diff=Output;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_respect_to_sigmainputoutput(Priors, Sigma,xi_r,d,Pointer_of_d_row,Pointer_of_d_colum,Pointer_of_K,Denominator,Numerator_P,Numerator_V)
Diff_P_handle=zeros(d,d);
Diff_P_handle(Pointer_of_d_row,Pointer_of_d_colum)=1;
Diff=0;
Input=xi_r;
% for i=1:d
%      Input(i,1)=xi_r(i,1);
% end
Diff=Diff+repmat((Priors(Pointer_of_K,1)*Numerator_P(:,Pointer_of_K).*Numerator_V(:,Pointer_of_K)./Denominator),1,d)'.*((Diff_P_handle/(Sigma(1:d,1:d,Pointer_of_K)))*(Input));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Constrain on A_k_Eigenvaue
function C=Comstrain_on_Ak(Qk,pointe_of_colum_and_row)
C=det(Qk(1:pointe_of_colum_and_row,1:pointe_of_colum_and_row));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Constrain on A_position_Symmetric
function Diff=Diff_constrains_ak_respect_to_sigmainput(Sigma_P,Sigma_V,P,d,~,Pointer_of_d_row,Pointer_of_d_colum,Pointer_of_K,pointer_of_colum_and_row,Pointer_of_KK)
Diff_P_handle=zeros(d,d);
Diff_P_handle(Pointer_of_d_row,Pointer_of_d_colum)=1;
Diff_P_handle(Pointer_of_d_colum,Pointer_of_d_row)=1;
Diff_P_handle=Diff_P_handle*sqrtm(Sigma_P(1:d,1:d,Pointer_of_K))+sqrtm(Sigma_P(1:d,1:d,Pointer_of_K))*Diff_P_handle;
output=0;
if Pointer_of_KK==Pointer_of_K
    A=[zeros(d,d) eye(d,d);Sigma_P(d+1:2*d,1:d,Pointer_of_K)/(Sigma_P(1:d,1:d,Pointer_of_K)) Sigma_V(d+1:2*d,1:d,Pointer_of_K)/(Sigma_V(1:d,1:d,Pointer_of_K))];
    handle=transpose(A)*P+P*A;
    handle2=handle(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row);
    A=[zeros(d,d) zeros(d,d);Sigma_P(d+1:2*d,1:d,Pointer_of_K)*(-Sigma_P(1:d,1:d,Pointer_of_K)\Diff_P_handle/Sigma_P(1:d,1:d,Pointer_of_K)) zeros(d,d)];
    handle3=transpose(A)*P+P*A;
    output=det(handle2)*trace(handle2\(handle3(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row)));
end
Diff=output;
%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_constrains_ak_respect_to_sigmainput_V(Sigma_P,Sigma_V,P,d,~,Pointer_of_d_row,Pointer_of_d_colum,Pointer_of_K,pointer_of_colum_and_row,Pointer_of_KK)
Diff_P_handle=zeros(d,d);
Diff_P_handle(Pointer_of_d_row,Pointer_of_d_colum)=1;
Diff_P_handle(Pointer_of_d_colum,Pointer_of_d_row)=1;
Diff_P_handle=Diff_P_handle*sqrtm(Sigma_V(1:d,1:d,Pointer_of_K))+sqrtm(Sigma_V(1:d,1:d,Pointer_of_K))*Diff_P_handle;
output=0;
if Pointer_of_KK==Pointer_of_K
    A=[zeros(d,d) eye(d,d);Sigma_P(d+1:2*d,1:d,Pointer_of_K)/(Sigma_P(1:d,1:d,Pointer_of_K)) Sigma_V(d+1:2*d,1:d,Pointer_of_K)/(Sigma_V(1:d,1:d,Pointer_of_K))];
    handle=transpose(A)*P+P*A;
    handle2=handle(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row);
    A=[zeros(d,d) zeros(d,d);zeros(d,d) Sigma_V(d+1:2*d,1:d,Pointer_of_K)*(-Sigma_V(1:d,1:d,Pointer_of_K)\Diff_P_handle/Sigma_V(1:d,1:d,Pointer_of_K))];
    handle3=transpose(A)*P+P*A;
    output=det(handle2)*trace(handle2\(handle3(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row)));
end
Diff=output;
%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_constrains_ak_respect_to_sigmainputoutput(Sigma_P,Sigma_V,P,d,Pointer_of_d_row,Pointer_of_d_colum,Pointer_of_K,pointer_of_colum_and_row,Pointer_of_KK)
Diff_P_handle=zeros(d,d);
Diff_P_handle(Pointer_of_d_row,Pointer_of_d_colum)=1;
output=0;
if Pointer_of_KK==Pointer_of_K
    A=[zeros(d,d) eye(d,d);Sigma_P(d+1:2*d,1:d,Pointer_of_K)/(Sigma_P(1:d,1:d,Pointer_of_K)) Sigma_V(d+1:2*d,1:d,Pointer_of_K)/(Sigma_V(1:d,1:d,Pointer_of_K))];
    handle=transpose(A)*P+P*A;
    handle2=handle(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row);
    A=[zeros(d,d) zeros(d,d);Diff_P_handle/(Sigma_P(1:d,1:d,Pointer_of_K)) zeros(d,d)];
    handle3=transpose(A)*P+P*A;
    output=det(handle2)*trace(handle2\(handle3(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row)));
end
Diff=output;
%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_constrains_ak_respect_to_sigmainputoutput_V(Sigma_P,Sigma_V,P,d,Pointer_of_d_row,Pointer_of_d_colum,Pointer_of_K,pointer_of_colum_and_row,Pointer_of_KK)
Diff_P_handle=zeros(d,d);
Diff_P_handle(Pointer_of_d_row,Pointer_of_d_colum)=1;
output=0;
if Pointer_of_KK==Pointer_of_K
    A=[zeros(d,d) eye(d,d);Sigma_P(d+1:2*d,1:d,Pointer_of_K)/(Sigma_P(1:d,1:d,Pointer_of_K)) Sigma_V(d+1:2*d,1:d,Pointer_of_K)/(Sigma_V(1:d,1:d,Pointer_of_K))];
    handle=transpose(A)*P+P*A;
    handle2=handle(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row);
    A=[zeros(d,d) zeros(d,d);zeros(d,d) Diff_P_handle/(Sigma_V(1:d,1:d,Pointer_of_K))];
    handle3=transpose(A)*P+P*A;
    output=det(handle2)*trace(handle2\(handle3(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row)));
end
Diff=output;
%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_constrains_ak_respect_to_P(Sigma_P,Sigma_V,P,d,Pointer_of_d_row,Pointer_of_d_colum,Pointer_of_K,pointer_of_colum_and_row,Pointer_of_KK)
Diff_P_handle=zeros(2*d,2*d);
Diff_P_handle(Pointer_of_d_row,Pointer_of_d_colum)=1;
Diff_P_handle(Pointer_of_d_colum,Pointer_of_d_row)=1;
output=0;
if Pointer_of_KK==Pointer_of_K
    A=[zeros(d,d) eye(d,d);Sigma_P(d+1:2*d,1:d,Pointer_of_K)/(Sigma_P(1:d,1:d,Pointer_of_K)) Sigma_V(d+1:2*d,1:d,Pointer_of_K)/(Sigma_V(1:d,1:d,Pointer_of_K))];
    handle=transpose(A)*P+P*A;
    handle2=handle(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row);
    handle3=transpose(A)*Diff_P_handle+Diff_P_handle*A;
    output=det(handle2)*trace(handle2\(handle3(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row)));
end
Diff=output;
%%%%%%%%%%%%%%%%%%%%%
function Diff=Diff_constrains_P_respect_to_P(P,d,Pointer_of_d_row,Pointer_of_d_colum,Pointer_of_K,pointer_of_colum_and_row,Pointer_of_KK)
Diff_P_handle=zeros(2*d,2*d);
Diff_P_handle(Pointer_of_d_row,Pointer_of_d_colum)=1;
Diff_P_handle(Pointer_of_d_colum,Pointer_of_d_row)=1;
output=0;
if Pointer_of_KK==Pointer_of_K
    handle2=P(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row,Pointer_of_K);
    handle3=Diff_P_handle;
    output=det(handle2)*trace(handle2\(handle3(1:pointer_of_colum_and_row,1:pointer_of_colum_and_row)));
end
Diff=output;
