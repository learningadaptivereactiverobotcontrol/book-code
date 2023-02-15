function [Priors_P,Mu_P,Sigma_P,Priors_V,Mu_V,Sigma_V,Turboshooting]=Initilizing_Second_Order(demos, Data_P_A, Data_V_A,Option)

K=Option.K;
dt=Option.dt;
tol_cutting=Option.tol_cutting;
I=Option.I;
%%%%%%% Method 1
if Option.Method==1
    [~ , ~, Data_A, ~] = preprocess_demos(demos,dt,tol_cutting);
    d=size(Data_A,1)/4;
        Data_A(3*d+1:end,:)=Data_A(3*d+1:end,:)/2;
%     Data_A=[Data_AA(1:d,:);Data_AA(d+1:2*d,:);Data_AA(3*d+1:4*d,:)];
    if I==1
       Data=Data_A;
    else
        Data=Data_A(:,1:ceil(size(Data_A,2)/I)+1);
    end
    [Priors_0, Mu_0, Sigma_0] = initialize_SEDS_One(Data,K); %finding an initial guess for GMM's parameter
    Priors_P=(Priors_0); 
    Priors_V=(Priors_0);
    Zero=zeros(d,d,K);  
    Mu_P=[Mu_0(1:d,:);Mu_0(3*d+1:4*d,:)]; 
    Mu_V=[(Mu_0(d+1:2*d,:));Mu_0(3*d+1:4*d,:)];
    Sigma_P=[Sigma_0(1:d,1:d,:) Zero;Sigma_0(3*d+1:4*d,1:d,:) Zero];
    Sigma_V=[Sigma_0(d+1:2*d,d+1:2*d,:) Zero; Sigma_0(3*d+1:4*d,d+1:2*d,:) Zero];
    counter=1;
    for i=1:K
        A(:,counter)=eig(Sigma_0(3*d+1:4*d,1:d,i)*inv(Sigma_0(1:d,1:d,i))); 
        counter=counter+1;
    end
    Sigma_V=[Sigma_0(d+1:2*d,d+1:2*d,:) Zero; Sigma_0(3*d+1:4*d,d+1:2*d,:) Zero];
    for i=1:K
        A(:,counter)=eig(Sigma_0(3*d+1:4*d,d+1:2*d,i)*inv(Sigma_0(d+1:2*d,d+1:2*d,i)));
                counter=counter+1;
    end
    if max(max(A))>0
        disp('Change the initialization method')
        Turboshooting=1;
    else
        Turboshooting=0;
    end
%%%%%%% Method 2
elseif Option.Method==2
    if I==1
       Data=Data_P_A;
    else
        Data=Data_P_A(:,1:ceil(size(Data_P_A,2)/I)+1);
    end
    d=size(Data_P_A,1)/2;
    Data(d+1:2*d,:)=Data(d+1:2*d,:)/2;
[Priors_P, Mu_P, Sigma_P] = initialize_SEDS(Data,K); 
clear Data
    if I==1
       Data=Data_V_A;
    else
        Data=Data_V_A(:,1:ceil(size(Data_V_A,2)/I)+1); 
    end
    Data(d+1:2*d,:)=Data(d+1:2*d,:)/2;
[Priors_V, Mu_V, Sigma_V] = initialize_SEDS(Data,K);
for k=1:K 
        Sigma_P(d+1:end,1:d,k) = -(abs(diag(Mu_P(d+1:2*d,k)./(inv(Sigma_P(1:d,1:d,k))*Mu_P(1:d,k)))));
        Sigma_V(d+1:end,1:d,k) = -(abs(diag(Mu_V(d+1:2*d,k)./(inv(Sigma_V(1:d,1:d,k))*Mu_V(1:d,k)))));
end
        counter=1;
    for i=1:K
        A(:,counter)=eig(Sigma_P(d+1:2*d,1:d,i)*inv(Sigma_P(1:d,1:d,i)));
        counter=counter+1;
    end
    for i=1:K
        A(:,counter)=eig(Sigma_V(d+1:2*d,1:d,i)*inv(Sigma_V(1:d,1:d,i)));
                counter=counter+1;
    end
    if max(max(A))>0
        disp('Change the initialization method')
        Turboshooting=1;
    else
        Turboshooting=0;
    end
end