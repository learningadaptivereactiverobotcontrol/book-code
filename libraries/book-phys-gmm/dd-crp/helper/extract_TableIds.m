function Z_C = extract_TableIds(C)

N = length(C);

K = 0;
Z_C = zeros(N,1);
for i = 1:N
  if Z_C(i)==0
    K = K+1;
    Piold = Z_C;
    curr = i;
    Z_C(curr) = K;
    while ~isequal(Z_C,Piold)
      Piold=Z_C;
      curr = C(curr);
      if curr>0
        if Z_C(curr)==0
          Z_C(curr) = K;
        elseif Z_C(curr)<K
          k = Z_C(curr);
          Z_C(Z_C==K) = k;
          K = K-1;
          break
        end
      end
    end
  end
end
