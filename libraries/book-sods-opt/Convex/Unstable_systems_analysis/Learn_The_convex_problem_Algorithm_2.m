function [prior_out_f,Mu_out_f,Sigma_out_f,A_f,Time]=Learn_The_convex_problem_Algorithm_2(prior, Mu, Sigma,Data)
K=size(Sigma,3);
d=size(Sigma,1)/2;
% options=sdpsettings('solver','quadprog','verbose',0);
options=sdpsettings('solver','CPLEX-IBM','verbose',0);
counter=2;
tol(1)=10^10;
tol(2)=10^6;
Sigma_out{counter}=Sigma;
prior_out{counter}=prior;
Mu_out{counter}=Mu;
Time=0;
while ((abs(tol(counter)-tol(counter-1))>0.01)&&(counter<20))
    if counter>2
        i=1;
        [prior, Mu, Sigma]=Solver_MSE_NoSigma(prior_out{counter-1}, Mu_out{counter-1},Sigma_out{counter-1},Data);

    end
    
    H=H_x(prior, Mu, Sigma,Data(1:d,:));
    A = sdpvar(d,d,K,'full');
    b= sdpvar(d,K,'full');
    Fun=0;
    if size(H,2)~=K
        keyboard
    end
    for i=1:K
        Fun=Fun+repmat(H(:,i),1,d)'.*(A(:,:,i)*Data(1:d,:)+repmat(b(:,i),1,size(Data,2)));
        %         Fun=Fun+repmat(H(:,i),1,d)'.*(A(:,:,i)*Data(1:d,:));
    end
    diff=Fun-Data(d+1:2*d,:);
    
    %     FUN=sum((sum(diff.^2)));
    %     sol =  optimize([],FUN,options);
    aux = sdpvar(2,length(diff));
    Fun=sum((sum(aux.^2)));
    sol=optimize([aux == diff],Fun,options);
    Fun=value(Fun);
    if sol.
        problem~=0
        keyboard
    end
    disp( sprintf('Value of FUN  %d %d ',[Fun counter]));
    %     disp( sprintf('Value of FUN  %d %d ',[value(FUN) counter]));
    tol(counter+1)=Fun;
    %     tol(counter+1)=value(FUN);
    Time=Time+sol.solvertime;
    % sol =  optimize([],sum(Fun));
    AA{counter} = value(A);
    if isnan(AA{counter})==1
        keyboard
    end
    bb{counter} = value(b);
    %     b=zeros(d,K);
    Sigma_out{counter}=Sigma;
    prior_out{counter}=prior;
    Mu_out{counter}=Mu;
    for i=1:K
        Sigma_input_output=AA{counter}(:,:,i)*Sigma(1:d,1:d,i);
        Sigma_out{counter}(d+1:2*d,1:d,i)=Sigma_input_output;
        Sigma_out{counter}(1:d,d+1:2*d,i)=Sigma_input_output';
        Mu_out{counter}(d+1:2*d,i)=AA{counter}(:,:,i)*Mu(1:d,i)+bb{counter}(:,i);
    end
    counter=counter+1;
end
Indext=find(tol==min(tol));
Indext=Indext(1,1)-1;
prior_out_f=prior_out{Indext};
Sigma_out_f=Sigma_out{Indext};
Mu_out_f=Mu_out{Indext};
A_f=AA{Indext};
Indext


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
