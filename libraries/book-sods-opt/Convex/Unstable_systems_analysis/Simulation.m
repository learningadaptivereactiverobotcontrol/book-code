function Simulation(Input_S,Unstable,Stable)

d=size(Stable.Mu)/2;
options = odeset('RelTol',1e-4,'AbsTol',1e-4);
for j=1:length(Input_S)
    Xrobot_object_initial=Input_S{1,j}(:,1);
    Xrobot_object_target=Input_S{1,j}(:,end);
[T1,x] = ode45(@(t,xi_r) SE(t,xi_r,Stable.prior,Stable.Mu,Stable.Sigma,Stable.A,Xrobot_object_target),[0:0.001:100],Xrobot_object_initial,options);
plot(x(:,1),x(:,2),'DisplayName','Generated trajectory','LineWidth',2,'Color',[0 0 1]);
end 
 
 
 function Dx_main = SE(t,xi_r,Priors,Mu,Sigma,A,Xrobot_object_target)
K=size(Sigma);
if max(size(K))==2
    K(1,3)=1;
end
d=size(xi_r);
Input=xi_r;

for i=1:K(1,3)
    Numerator(:,i)=gaussPDF(Input, Mu(1:d(1,1),i), Sigma(1:d(1,1),1:d(1,1),i));
    Pxi(:,i) = Priors(i).*Numerator(:,i)+realmin;
end
Denominator=sum(Pxi,2)+realmin;
beta = Pxi./repmat(Denominator,1,K(1,3));
for j=1:K(1,3)
%      b=Mu(d(1,1)+1:2*d(1,1),j)-A{j}*Mu(1:d(1,1),j);
     y_tmp(:,:,j) =  A{1,j}* (Input-Xrobot_object_target);
end
beta_tmp = reshape(beta,[1 size(beta)]);
y_tmp2 = repmat(beta_tmp,[length(d(1,1)+1:2*d(1,1)) 1 1]) .* y_tmp;
y = sum(y_tmp2,3);

Dx_main=y;