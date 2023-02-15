function [P,F,Sigma_P,Sigma_V]=P_Calculator(Sigma_P,Sigma_V)

d=size(Sigma_P,1)/2;
K=size(Sigma_P);
if max(size(K))==2
    K(1,3)=1;
end
K=K(1,3);

for i=K
    Sigma_P(:,:,i)=Sigma_P(:,:,1);
    Sigma_V(:,:,i)=Sigma_V(:,:,1);
end
for i=1:K
    A(:,:,i)=[zeros(d,d) eye(d,d);Sigma_P(d+1:2*d,1:d,i)/(Sigma_P(1:d,1:d,i)) Sigma_V(d+1:2*d,1:d,i)/(Sigma_V(1:d,1:d,i))];
end

F=1;
P = sdpvar(2*d,2*d);
C=[];
for i=1:K
    C=C+[A(:,:,i)'*P+P*A(:,:,i) <= -0.0000001*eye(2*d,2*d)];
end
C=C+[0.000001*eye(2*d,2*d)<=P];
Fun=0;
options=sdpsettings('solver','sedumi','verbose',0);
sol =  optimize(C,Fun,options);
P = value(P);
for i=1:K
    A(:,:,i)
    P
eig(A(:,:,i)'*P+P*A(:,:,i))
end
if (sol.problem~=0)
    F=0;
end













function [C, Ceq, Grad_C, Grad_Ceq]=ctr_eigenvalue(x,d,A,K)
[P] = shape_DS(x,d);

%%%%%%%%%%%%%%%%%%%%%   Constrain on A
C=zeros(1,d+d*K);
Ceq=[];
Grad_Ceq=[];
Grad_C=[];
counter=1;
for pointer_of_colum_and_row=1:d
    C(counter)=-Constrain_on_Ak(P,pointer_of_colum_and_row);
    counter=counter+1;
end
for i=1:K
    for pointer_of_colum_and_row=1:d
        C(counter)=(-1)^(pointer_of_colum_and_row+1)*Constrain_on_Ak(transpose(A(:,:,i))*P+P*A(:,:,i),pointer_of_colum_and_row);
        counter=counter+1;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X] = shape_DS(p,d)

X = zeros(d,d);
hadle=1;
for j=1:d
    for i=j:d
        X (i,j)= p(hadle);
        X (j,i)= p(hadle);
        hadle=hadle+1;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x1 = To_Parameters(Input,d)
hadle=1;
for j=1:d
    for i=j:d
        x1(hadle) = Input(i,j);
        hadle=hadle+1;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function obj_handle = obj(p,d,A,K)
[P] = shape_DS(p,d);
obj_handle=0;
for i=1:K
    % obj_handle=obj_handle+norm(norm(transpose(A(:,:,i))*P+P*A(:,:,i)+eye(d,d)));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function C=Constrain_on_Ak(P,pointe_of_colum_and_row)
output=P;
C=det(output(1:pointe_of_colum_and_row,1:pointe_of_colum_and_row));
