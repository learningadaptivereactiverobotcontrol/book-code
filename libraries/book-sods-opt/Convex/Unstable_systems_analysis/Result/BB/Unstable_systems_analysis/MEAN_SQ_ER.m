function ERROR=MEAN_SQ_ER(Data,prior,Mu,Sigma,A)

d=size(Sigma,1)/2;
K=size(Sigma,3);

X_Input=Data(1:d,:);
DX_out = SE(X_Input,prior,Mu,Sigma,A);
diff=Data(d+1:2*d,:)-DX_out;
% for i=1:size(Data,2)
%     Fun(i)=norm(diff(:,i));
% end
ERROR=sum(sqrt(sum(diff.^2)));


function Dx_main = SE(xi_r,Priors,Mu,Sigma,A)
K=size(Sigma);
if max(size(K))==2
    K(1,3)=1;
end
d=size(xi_r);
Input=xi_r;

for i=1:K(1,3)
    Numerator(:,i)=gaussPDF(Input, Mu(1:d(1,1),i), Sigma(1:d(1,1),1:d(1,1),i));
    Pxi(:,i) = Priors(i).*Numerator(:,i);
end
Denominator=sum(Pxi,2)+realmin;
beta = Pxi./repmat(Denominator,1,K(1,3));
for j=1:K(1,3)
     b=Mu(d(1,1)+1:2*d(1,1),j)-A(:,:,j)*Mu(1:d(1,1),j);
     y_tmp(:,:,j) =  A(:,:,j)* (Input)+repmat(b,1,size(xi_r,2));
% y_tmp(:,:,j) =  A(:,:,j)* (Input);
end
beta_tmp = reshape(beta,[1 size(beta)]);
y_tmp2 = repmat(beta_tmp,[length(d(1,1)+1:2*d(1,1)) 1 1]) .* y_tmp;
y = sum(y_tmp2,3);

Dx_main = sum(y_tmp2,3);