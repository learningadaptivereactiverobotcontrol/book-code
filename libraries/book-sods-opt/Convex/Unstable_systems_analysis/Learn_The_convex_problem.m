function [prior_out,Mu_out,Sigma_out,A,Time]=Learn_The_convex_problem(prior, Mu, Sigma,Data)
K=size(Sigma,3);
d=size(Sigma,1)/2;
options=sdpsettings('solver','CPLEX-IBM','verbose',0);
H=H_x(prior, Mu, Sigma,Data(1:d,:));
A = sdpvar(d,d,K,'full');
b= sdpvar(d,K,'full');
Fun=0;
for i=1:K
    Fun=Fun+repmat(H(:,i),1,d)'.*(A(:,:,i)*Data(1:d,:)+repmat(b(:,i),1,size(Data,2)));
end
diff=Fun-Data(d+1:2*d,:);

% FUN=sum((sum(diff.^2)));
% tic
% sol =  optimize([],FUN,options);
% Time=toc;
aux = sdpvar(d,length(diff));
Fun=sum((sum(aux.^2)));

tic
sol=optimize([aux == diff],Fun,options);
Time=toc;
Fun=value(Fun);
disp( sprintf('Value of FUN  %d %d ',[Fun]));
% Time=sol.solvertime;
% sol =  optimize([],sum(Fun));
A = value(A);
b = value(b);
% b=zeros(d,K);
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

function plot_K_Mean_DATA(Data,Mu_out)
K=size(Mu_out,2);
d=size(Mu_out,1)/2;
i=1;
temp_h=repmat(Mu_out(1:d,i),1,size(Data(1:d,:),2))-Data(1:d,:);
temp_h=sum(temp_h.^2,1);
index=ones(1,size(Data(d+1:2*d,:),2));
temp=temp_h;
for i=2:K
    temp_h=repmat(Mu_out(1:d,i),1,size(Data(1:d,:),2))-Data(1:d,:);
    temp_h=sum(temp_h.^2,1);
    index(temp_h<temp)=i;
    temp(temp_h<temp)=temp_h(temp_h<temp);
end
figure();
hold on
for i=1:K
    plot(Mu_out(1,i),Mu_out(2,i),'+','color',[i/K 1-i/K (i/K)^2])
    plot(Data(1,index==i),Data(2,index==i),'.','color',[i/K 1-i/K (i/K)^2])
end