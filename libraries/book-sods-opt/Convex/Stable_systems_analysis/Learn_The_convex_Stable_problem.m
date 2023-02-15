function [prior_out,Mu_out,Sigma_out,A,Time]=Learn_The_convex_Stable_problem(prior, Mu, Sigma,Data)
K=size(Sigma,3);
d=size(Sigma,1)/2;
tol=10^(-4);
options=sdpsettings('solver','sedumi','verbose',0);
H=H_x(prior, Mu, Sigma,Data(1:d,:));
P0=eye(d,d);

Method=1;
if Method==1
    A = sdpvar(d,d,K,'full');
    C=[];
    for i=1:K
        C=C+[A(:,:,i)'+A(:,:,i) <= -0.0001*eye(d,d)];
    end
    Fun=0;
    for i=1:K
        %     Fun=Fun+repmat(H(:,i),1,d)'.*(A(:,:,i)*Data(1:d,:)+repmat(b(:,i),1,size(Data,2)));
        Fun=Fun+repmat(H(:,i),1,d)'.*(A(:,:,i)*Data(1:d,:));
    end
    diff=Fun-Data(d+1:2*d,:);
    
    % FUN=sum((sum(diff.^2)));
    aux = sdpvar(d,length(diff));
    Fun=sum((sum(aux.^2)));
    C=C+[aux == diff];
    sol =  optimize(C,Fun);
    if sol.problem~=0
        disp('PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM')
        K
    end
    Time=sol.solvertime;
    % sol =  optimize([],sum(Fun));
    A = value(A);
    % b = value(b);
    b=zeros(d,K);
    Sigma_out=Sigma(1:d,1:d,:);
    prior_out=prior;
    Mu_out=Mu(1:d,:);
elseif Method==2
    A = sdpvar(d,d,K,'full');
    P = sdpvar(d,d);
    C=[];
    for i=1:K
        C=C+[A(:,:,i)'*P+P*A(:,:,i) <= -0.0001*eye(d,d)];
    end
    C=C+[0.0001*eye(d,d)<=P];
    Fun=0;
    for i=1:K
        %     Fun=Fun+repmat(H(:,i),1,d)'.*(A(:,:,i)*Data(1:d,:)+repmat(b(:,i),1,size(Data,2)));
        Fun=Fun+repmat(H(:,i),1,d)'.*(A(:,:,i)*Data(1:d,:));
    end
    diff=Fun-Data(d+1:2*d,:);
    % FUN=sum((sum(diff.^2)));
    aux = sdpvar(d,length(diff));
    Fun=sum((sum(aux.^2)));
    C=C+[aux == diff];
    sol =  optimize(C,Fun);
    if sol.problem~=0
        disp('PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM PROBLEM')
        K
    end
    Time=sol.solvertime;
    % sol =  optimize([],sum(Fun));
    A = value(A);
    P=value(P)
    % b = value(b);
    b=zeros(d,K);
    Sigma_out=Sigma(1:d,1:d,:);
    prior_out=prior;
    Mu_out=Mu(1:d,:);
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