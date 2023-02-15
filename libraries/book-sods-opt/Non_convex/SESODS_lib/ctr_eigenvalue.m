function [C, Ceq, Grad_C, Grad_Ceq]=ctr_eigenvalue(x,d,K,EPSILON,Sigma_input_P,Sigma_input_V)
epsilon=EPSILON;
[~, ~, Sigma_P , ~, Sigma_V,P] = shape_DS(x,d,K);
% P=[12.8109151512572,0,0.0172490488302238,0;0,1.32331221994862,0,0.216761990831361;0.0172490488302238,0,0.450483014739464,0;0,0.216761990831361,0,0.242446384970951];
if Sigma_P(1,1,1)==0
    Sigma_P(1:d,1:d,:)=Sigma_input_P;
    Sigma_V(1:d,1:d,:)=Sigma_input_V;
end
% Sigma_V=[0.188297642746613,0,-0.210924652015895,0;0,0.0250221067571891,0,-0.0732190094821703;-0.210924652015895,0,0,0;0,-0.0732190094821703,0,0];
Ceq=[];
Grad_Ceq=[];
Grad_C=[];
%%%%%%%%%%%%%%%%%%%%%   Constrain on A
% C=zeros(1,2*K*d+2*K*d);
C=zeros(1,2*K*d+2*K*d);
counter=1;
for pointer_of_K=1:K
    A=[zeros(d,d) eye(d,d);Sigma_P(d+1:2*d,1:d,pointer_of_K)/(Sigma_P(1:d,1:d,pointer_of_K)) Sigma_V(d+1:2*d,1:d,pointer_of_K)/(Sigma_V(1:d,1:d,pointer_of_K))];
    for pointer_of_colum_and_row=1:2*d
        C(counter)=((-1)^(pointer_of_colum_and_row+1))*Comstrain_on_Ak(transpose(A)*P+P*A,pointer_of_colum_and_row)+epsilon;
        counter=counter+1;
    end
end
for pointer_of_colum_and_row=1:2*d
    C(counter)=-Comstrain_on_Ak(P(:,:),pointer_of_colum_and_row)+epsilon;
    counter=counter+1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Derivatives
C

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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Constrain on A_k_Eigenvaue
function C=Comstrain_on_Ak(Qk,pointe_of_colum_and_row)
C=det(Qk(1:pointe_of_colum_and_row,1:pointe_of_colum_and_row));
