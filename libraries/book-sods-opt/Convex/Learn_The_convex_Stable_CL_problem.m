function [prior_out,Mu_out,Sigma_out,A,Time]=Learn_The_convex_Stable_CL_problem(prior, Mu, Sigma,Data)
K=size(Sigma,3);
d=size(Sigma,1)/2;
options=sdpsettings('solver','sedumi','verbose',0);
H=H_x(prior, Mu, Sigma,Data(1:d,:));
A = sdpvar(d,d,K,'full');
K_p=100;
delta=100;
C=[];
for i=1:K
    handle=[-K_p*eye(d,d),zeros(d,d);delta*eye(d,d), A(:,:,i)];
    C=C+[handle'+handle <= -0.0001*eye(2*d,2*d)];
end
Fun=0;
for i=1:K
    %     Fun=Fun+repmat(H(:,i),1,d)'.*(A(:,:,i)*Data(1:d,:)+repmat(b(:,i),1,size(Data,2)));
    Fun=Fun+repmat(H(:,i),1,d)'.*(A(:,:,i)*Data(1:d,:));
end
diff=Fun-Data(d+1:2*d,:);

FUN=sum((sum(diff.^2)));
t = sdpvar(1);
C=C+[FUN<= t];
sol =  optimize(C,t,options);
if sol.problem~=0
    disp('PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM')
    K
end
Time=sol.solvertime;
% sol =  optimize([],sum(Fun));
A = value(A);
% b = value(b);
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