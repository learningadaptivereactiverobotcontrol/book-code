function [ERROR]=Cross_Validation(prior,Mu,Sigma,A,b,Data)
K=size(Sigma,3);
d=size(Sigma,1)/2;
H=H_x(prior, Mu, Sigma,Data(1:d,:));
Fun=0;
for i=1:K
    Fun=Fun+repmat(H(:,i),1,d)'.*(A(:,:,i)*Data(1:d,:)+repmat(b(:,i),1,size(Data,2)));
end
diff=Fun-Data(d+1:2*d,:);
ERROR=sum(sqrt(sum(diff.^2)));


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