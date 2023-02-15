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
