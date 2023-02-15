function DX_out=Simulate_stream_line(prior,Mu,Sigma,A,D)


nx=500;
ny=500;
ax.XLim = D(1,1:2);
ax.YLim = D(1,3:4);
scale=1;
ax_x=linspace(scale*ax.XLim(1),scale*ax.XLim(2),nx); %computing the mesh points along each axis
ax_y=linspace(scale*ax.YLim(1),scale*ax.YLim(2),ny); %computing the mesh points along each axis
[x_tmp, y_tmp]=meshgrid(ax_x,ax_y); %meshing the input domain
x=[x_tmp(:) y_tmp(:)]';

DX_out = SE(x,prior,Mu,Sigma,A);

streamslice(reshape(x(1,:),nx,ny),reshape(x(2,:),nx,ny),reshape(DX_out(1,:),nx,ny),reshape(DX_out(2,:),nx,ny),1,'method','cubic');

function Dx_main = SE(xi_r,Priors,Mu,Sigma,A)
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
     b=Mu(d(1,1)+1:2*d(1,1),j)-A(:,:,j)*Mu(1:d(1,1),j);
     y_tmp(:,:,j) =  A(:,:,j)* (Input)+repmat(b,1,size(xi_r,2));
    %   y_tmp(:,:,j) =  (x);
end
beta_tmp = reshape(beta,[1 size(beta)]);
y_tmp2 = repmat(beta_tmp,[length(d(1,1)+1:2*d(1,1)) 1 1]) .* y_tmp;
y = sum(y_tmp2,3);

Dx_main=y;