function [prior_out,Mu_out,Sigma_out,A,Time]=Learn_The_convex_Stable_problem_second(prior, Mu, Sigma,Data)
K=size(Sigma,3);
d=size(Sigma,1)/4;
tol=10^(-4);
options=sdpsettings('verbose',1);
H=H_x(prior, Mu, Sigma,Data(1:2*d,:));
P0=eye(d,d);

for i=1:K
    A{i} = [zeros(d,d)  eye(d,d); sdpvar(d,d,'full') sdpvar(d,d,'full')];
end
P = sdpvar(2*d,2*d);
C=[];
for i=1:K
    C=C+[transpose(A{i})*P+P*A{i}<= -0.0001*eye(2*d,2*d)];
end
C=C+[0.1*eye(2*d,2*d)<=P];
Fun=0;
for i=1:K
    %     Fun=Fun+repmat(H(:,i),1,d)'.*(A(:,:,i)*Data(1:d,:)+repmat(b(:,i),1,size(Data,2)));
    Fun=Fun+repmat(H(:,i),1,2*d)'.*(A{i}*Data(1:2*d,:));
end
diff=Fun(d+1:2*d,:)-Data(3*d+1:4*d,:);
% FUN=sum((sum(diff.^2)));
aux = sdpvar(d,length(diff));
Fun=sum((sum(aux.^2)));
C=C+[aux == diff];
sol =  optimize(C,Fun,options);
if sol.problem~=0
    disp('PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM')
    K
end
Time=sol.solvertime;
% sol =  optimize([],sum(Fun));
for i=1:K
   A{i} = value(A{i});  
end
P=value(P)
eig(P)
b=zeros(d,K);
Sigma_out=Sigma(1:2*d,1:2*d,:);
prior_out=prior;
Mu_out=Mu(1:2*d,:);








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