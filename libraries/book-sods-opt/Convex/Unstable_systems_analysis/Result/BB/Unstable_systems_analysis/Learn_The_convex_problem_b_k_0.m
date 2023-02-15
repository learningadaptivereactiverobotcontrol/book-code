function [prior_out,Mu_out,Sigma_out,A]=Learn_The_convex_problem_b_k_0(prior, Mu, Sigma,Data)
K=size(Sigma,3);
d=size(Sigma,1)/2;
% options =sdpsettings('lmirank.solver','sedumi','sedumi.eps',0,'verbose',0);
options=sdpsettings('solver','quadprog');
H=H_x(prior, Mu, Sigma,Data(1:d,:));
A = sdpvar(d,d,K,'full');
Fun=0;
for i=1:K
    Fun=Fun+repmat(H(:,i),1,d)'.*(A(:,:,i)*Data(1:d,:));
end
diff=Fun-Data(d+1:2*d,:);
% clearvars  Fun
% for i=1:size(Data,2)
%     Fun(i)=norm(diff(:,i));
% end
FUN=sum((sum(diff.^2)));
sol =  optimize([],FUN,options);
% sol =  optimize([],sum(Fun));
A = value(A);
b=zeros(d,K);
Sigma_out=Sigma;
prior_out=prior;
Mu_out=Mu;
for i=1:K
    Sigma_input_output=A(:,:,i)*Sigma(1:d,1:d,i);
    Sigma_out(d+1:2*d,1:d,i)=Sigma_input_output;
    Sigma_out(1:d,d+1:2*d,i)=Sigma_input_output';
    Mu_out(d+1:2*d,i)=A(:,:,i)*Mu(1:d,i)+b(:,i);
end
%% Calculate the covariance of out-put
% method=1; %% Simple answer
% method=2; %% Minimize the Convariance of the output.
method=3;
if method==1
    Sigma_out_out = sdpvar(d,d,K);
    C=[];
    epsilon=10^(-8);
    for i=1:K
        Sigma_input_output=Sigma_out(d+1:2*d,1:d,i);
        Sigma_input=Sigma_out(1:d,1:d,i);
        C=C+[Sigma_out_out(:,:,i)-Sigma_input_output*inv(Sigma_input)*Sigma_out(1:d,d+1:2*d,i)-epsilon*eye(d,d)>=0];
    end
    sol =  optimize(C,[]);
    if sol.problem == 0
        Sigma_out_out=value(Sigma_out_out);
    else
        keyboard
    end
    Sigma_out(d+1:2*d,d+1:2*d,:)=Sigma_out_out;
elseif (method==2)||(method==3)
    Sigma_out_out = sdpvar(d,d,K);
    C=[];
    epsilon=10^(-4);
    for i=1:K
        Sigma_input_output=Sigma_out(d+1:2*d,1:d,i);
        Sigma_input=Sigma_out(1:d,1:d,i);
        C=C+[Sigma_out_out(:,:,i)-Sigma_input_output*inv(Sigma_input)*Sigma_out(1:d,d+1:2*d,i)-epsilon*eye(d,d)>=0];
    end
    F=0;
    for i=1:K
        Sigma_input_output=Sigma_out(d+1:2*d,1:d,i);
        Sigma_input=Sigma_out(1:d,1:d,i);
        F=F+norm(Sigma_out_out(:,:,i)-Sigma_input_output*inv(Sigma_input)*Sigma_out(1:d,d+1:2*d,i));
    end
    sol =  optimize(C,F);
    if sol.problem == 0
        Sigma_out_out=value(Sigma_out_out);
    else
        keyboard
    end
    Sigma_out(d+1:2*d,d+1:2*d,:)=Sigma_out_out;
    if method==3
        [prior_out,Mu_out,Sigma_out]=EM_maximization_Sigma_out(Data,prior_out,Mu_out,Sigma_out);
    end
end



% for i=1:K
%     Sigma_input_output=Sigma_out(d+1:2*d,1:d,i);
%     Sigma_input=Sigma_out(1:d,1:d,i);
%     eig(Sigma_out(d+1:2*d,d+1:2*d,i)-Sigma_out(d+1:2*d,1:d,i)*inv(Sigma_out(1:d,1:d,i))*Sigma_out(1:d,d+1:2*d,i))
%     eig(Sigma_out(:,:,i))
% end


function beta=H_x(prior, Mu, Sigma,Data)
K=size(Sigma);
if max(size(K))==2
    K(1,3)=1;
end
d=size(Data);
Input=Data;

for i=1:K(1,3)
    Numerator(:,i)=gaussPDF(Input, Mu(1:d(1,1),i), Sigma(1:d(1,1),1:d(1,1),i));
    Pxi(:,i) = prior(i).*Numerator(:,i)+realmin;
end
Denominator=sum(Pxi,2)+realmin;
beta = Pxi./repmat(Denominator,1,K(1,3));