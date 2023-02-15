function [ddgk, userdata] = bex3_conhess(x,Y,k,userdata)
  
  switch(k)
  case (1)
    ddgk = [...
        0 x(3)*x(4) x(2)*x(4) x(2)*x(3) 0 0;
        x(3)*x(4) 0 x(1)*x(4) x(1)*x(3) 0 0;
        x(2)*x(4) x(1)*x(4) 0 x(1)*x(2) 0 0;
        x(2)*x(3) x(1)*x(3) x(1)*x(2) 0 0 0;
        0 0 0 0 0 0;
        0 0 0 0 0 0];
  case (2)
    ddgk = [...
        2 0 0 0 0 0;
        0 2 0 0 0 0;
        0 0 2 0 0 0;
        0 0 0 2 0 0;
        0 0 0 0 0 0;
        0 0 0 0 0 0];
  end