function [F] = springFun(x)

[n,~]  = size(x);
T      = (n - 2) / 3;
ObjRow = 2*T + 1;

lin = 1;  nln = lin + T;
jy0 = 1;  jx0 = jy0 + T + 1; ju0 = jx0 + T + 1;

F = zeros(2*T+1,1);
fObj = 0;
for jt = 0:T-1
  jy = jy0 + jt;   jx = jx0 + jt;   ju = ju0 + jt;

  yt = x(jy); ytp1 = x(jy+1);

  xt = x(jx); xtp1 = x(jx+1);

  ut = x(ju);

  F(nln) = 1e-2 * yt*yt - yt + ytp1 + 4e-3*xt - .2*ut;
  F(lin) = -.2*yt - xt + xtp1;
  fObj   = fObj + xt*xt;

  nln    = nln + 1;   lin    = lin + 1;
end

F(ObjRow) = (fObj + xtp1*xtp1)/2;