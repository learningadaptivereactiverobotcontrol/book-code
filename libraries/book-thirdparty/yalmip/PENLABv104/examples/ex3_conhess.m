function [ddgk, userdata] = ex3_conhess(x,Y,k,userdata)
% Hessians of the constraints, Example 3

  switch(k)
  case (1)
    ddgk = [2, 0; 0, 0];
  case (2)
    ddgk = [0, 0; 0, 2];
  end

