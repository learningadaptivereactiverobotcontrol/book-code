

e=1e-8;k=18;

x=pp.x;
%x=ones(18,1);

a=tto_mconfun(x,[],1,pp.userdata);
xn=x; xn(k) = xn(k)+e;
an=tto_mconfun(xn,[],1,pp.userdata);

d = (an-a)/e;

dd=tto_mcongrad(x,[],1,k,pp.userdata);



e=1e-8;k=1;j=1;
a=tto_mcongrad(x,[],1,k,pp.userdata);
xn=x; xn(j) = xn(j)+e;
an=tto_mcongrad(xn,[],1,k,pp.userdata);

h = (an-a)/e;

hh=tto_mconhess(x,[],1,k,j,pp.userdata);