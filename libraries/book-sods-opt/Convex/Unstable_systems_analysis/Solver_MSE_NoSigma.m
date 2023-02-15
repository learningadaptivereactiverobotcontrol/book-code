function [Priors_P, Mu_P, Sigma_P]=Solver_MSE_NoSigma(Priors_0_P,Mu_0_P,Sigma_0_P,Data_A)
d = size(Sigma_0_P,1)/2; %dimension of the model
K = size(Sigma_0_P,3);


optNLP = optimset('Display', 'off','TolX',1e-6, 'TolFun', 1e-6, ...
    'MaxFunEval', 5000, 'MaxIter', 200000,'FinDiffRelStep',realmin ); %,'FinDiffType','central''DerivativeCheck','on' interior-point


p0 = GMM_2_Parameters(Mu_0_P,d,K);

obj_handle = @(p)obj((p),Data_A,Priors_0_P,Mu_0_P(d+1:2*d,:),Sigma_0_P,d,K);

[popt,fval,~] = fminsearch(obj_handle, p0,optNLP);
disp(sprintf('The value of the objective function is %d:',fval))
[Mu_0] = shape_DS(popt,d,K);
Mu_P=[Mu_0;Mu_0_P(d+1:2*d,:)];

Priors_P=Priors_0_P;
Sigma_P=Sigma_0_P;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Priors_P, Mu_P, Sigma_P]=Stabilizing_B_K(Priors_P,Mu_P,Sigma_P,d,K)
for pointe_of_K=1:K
    Sigmainput=Sigma_P(1:d,1:d,pointe_of_K);
    SigmaInput_Output=Sigma_P((d+1):2*d,1:d,pointe_of_K);
    Muinput=Mu_P(1:d,pointe_of_K);
    Mu_P(d+1:2*d,pointe_of_K)=(SigmaInput_Output/(Sigmainput))*Muinput;
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
function [J, DJ_Main]=obj(p,Data,Priors_0_P,Mu_0_P_out,Sigma_0_P,d,K)

% This function computes the derivative of the likelihood objective function
% w.r.t. optimization parameters.
DJ_Main=[];

[Mu_0_P] = shape_DS(p,d,K);
Mu_0=[Mu_0_P;Mu_0_P_out];
[DDX,~]= SE_DS(Priors_0_P,Mu_0,Sigma_0_P,Data(1:d,:));
diff=Data(d+1:2*d,:)-DDX;
ERROR=sum(sum(diff.^2));


J=ERROR;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Dx_main,Denominator]= SE_DS(Priors,Mu,Sigma,X)

K=size(Sigma);
if max(size(K))==2
    K(1,3)=1;
end
d=size(X);
Input=X;
Numerator=zeros(d(1,2),K(1,3));
Pxi=zeros(size(X,2),K(1,3));
y_tmp=zeros(d(1,1),size(X,2),K(1,3));

for i=1:K(1,3)
    Numerator(:,i)=gaussPDF(Input, Mu(1:d(1,1),i), Sigma(1:d(1,1),1:d(1,1),i));
    Pxi(:,i) = abs(Priors(i)).*Numerator(:,i)+realmin;
end
Denominator=sum(Pxi,2)+realmin;
beta = Pxi./repmat(Denominator,1,K(1,3));
for j=1:K(1,3)
    Sigmainput_P=Sigma(1:d(1,1),1:d(1,1),j);
    SigmaInput_Output_P=Sigma(d(1,1)+1:2*d(1,1),1:d(1,1),j);
    A=SigmaInput_Output_P/(Sigmainput_P);
    b=Mu(d(1,1)+1:2*d(1,1),j)-A*Mu(1:d(1,1),j);
    y_tmp(:,:,j) =  A* (Input)+repmat(b,1,size(Input,2));
end
beta_tmp = reshape(beta,[1 size(beta)]);
y_tmp2 = repmat(beta_tmp,[length(d(1,1)+1:2*d(1,1)) 1 1]) .* y_tmp;
Dx_main = sum(y_tmp2,3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Mu_0_P] = shape_DS(p,d,K)
% transforming the column of parameters into Priors, Mu, and Sigma
Mu_0_P = zeros(d,K);
hadle=0;
for k=1:K
    Mu_0_P(1:d,k) = reshape(p(hadle+1:hadle+d),d,1);
    hadle=hadle+d;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function p0 = GMM_2_Parameters(Mu_0_P,d,K)
x1=zeros(1,(K+d*K));
hadle=1;
for k=1:K
    x1(hadle:hadle+d-1) = reshape(Mu_0_P(1:d,k),1,d);
    hadle=hadle+d;
end
p0=x1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%