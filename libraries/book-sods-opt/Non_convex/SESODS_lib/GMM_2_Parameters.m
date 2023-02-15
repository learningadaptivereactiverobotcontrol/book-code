function p0 = GMM_2_Parameters(Priors_0_P,Mu_0_P,Sigma_0_P, Mu_0_V, Sigma_0_V,P,d,K)
x1=zeros(1,(K+d*K+d*(3*d+1)*K/2));
x2=zeros(1,(d*K+d*(3*d+1)*K/2));
x3=zeros(1,(d*(2*d+1)*K));

for i=1:(K)
    x1(i)=Priors_0_P(1,i);
end
hadle=i+1;
for k=1:K
x1(hadle:hadle+d-1) = reshape(Mu_0_P(1:d,k),1,d);
hadle=hadle+d;
end
for k=1:K
    for j=1:d
        for i=j:2*d
             x1(hadle) = Sigma_0_P(i,j,k);
             hadle=hadle+1;
        end
    end
end
hadle=1;
for k=1:K
x2(hadle:hadle+d-1) = reshape(Mu_0_V(1:d,k),1,d);
hadle=hadle+d;
end

for k=1:K
    for j=1:d
        for i=j:2*d
             x2(hadle) = Sigma_0_V(i,j,k);
             hadle=hadle+1;
        end
    end
end
hadle=1;
    for j=1:2*d
        for i=j:2*d
             x3(hadle) = P(i,j);
             hadle=hadle+1;
        end
    end
p0=[x1 x2 x3]; 