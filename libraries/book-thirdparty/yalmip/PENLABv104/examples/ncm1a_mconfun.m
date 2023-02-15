function [Akx, userdata] = mcm1a_mconfun(x,Y,k,userdata)
% There is only one matrix variable A(x) = sum x(i)*userdata.A{i}

  A=userdata.A;
  Akx=x(1).*A{1} + x(2).*A{2} + x(3).*A{3} + x(4).*A{4} + x(5).*A{5};

