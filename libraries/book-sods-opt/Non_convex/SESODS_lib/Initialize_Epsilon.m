function [C]=Initialize_Epsilon(Sigma_P,Sigma_V,K,d)
%%%%%%%   Constrain on A
counter=1;
for pointer_of_K=1:K
    for pointer_of_colum_and_row=1:d
        C(counter)=((-1)^(pointer_of_colum_and_row+1))*Comstrain_on_Ak(Sigma_P,d,pointer_of_K,pointer_of_colum_and_row);
        counter=counter+1;
    end 
end  
for pointer_of_K=1:K
    for pointer_of_colum_and_row=1:d
        C(counter)=((-1)^(pointer_of_colum_and_row+1))*Comstrain_on_Ak(Sigma_V,d,pointer_of_K,pointer_of_colum_and_row);
        counter=counter+1;
    end 
end  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Constrain on A_k_Eigenvaue
function C=Comstrain_on_Ak(Sigma,d,pointer_of_K,pointe_of_colum_and_row)
output=Sigma((d+1):2*d,1:d,pointer_of_K)/Sigma(1:d,1:d,pointer_of_K);
C=det(output(1:pointe_of_colum_and_row,1:pointe_of_colum_and_row)+transpose(output(1:pointe_of_colum_and_row,1:pointe_of_colum_and_row)));
