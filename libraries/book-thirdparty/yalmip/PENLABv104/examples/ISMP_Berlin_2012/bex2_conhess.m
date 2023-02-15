function [ddgk, userdata] = bex2_conhess(x,Y,k,userdata)
  
  switch(k)
  case (1)
    ddgk = [2 0 0 0; 0 2 0 0; 0 0 2 0; 0 0 0 2];
  case (2)
    ddgk = [2 0 0 0; 0 4 0 0; 0 0 2 0; 0 0 0 4];
  case (3)
    ddgk = [4 0 0 0; 0 2 0 0; 0 0 2 0; 0 0 0 0];
  end