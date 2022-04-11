function [ h ] = soft_step_fun(upper, t, target, cof)

a = 1 + cof/2;
k = (-1/upper) * log((-2+cof)/(2+cof));

% Computing norm of distance
t = sqrt(sum(((t-target).^2),1));

h=zeros(size(t));
h(t==0) = 1;
if (norm(size(h(t<=upper)))~=0)
    h(t<=upper) = -cof./(1+exp(-k*t(t<=upper))) + a;
end
h(upper<t)=0;
end

