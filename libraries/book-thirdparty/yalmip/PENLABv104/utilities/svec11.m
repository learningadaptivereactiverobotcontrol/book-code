function [evec] = svec1(emat)

    [m,n]= size(emat);
    idx = 1;
    for j=1:m
       for k=1:j
          if j==k
              evec(idx) = emat(j,k);
          else
              evec(idx) = emat(j,k);
          end
          idx = idx + 1;
       end
    end

end