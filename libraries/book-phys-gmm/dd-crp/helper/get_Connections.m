function [customers] = get_Connections(C,i)
% This function returns customers that are connected to i, directly or indirectly
% does the job of the sitbehind(i) function from the Socher11 paper.

nn = 0;
customers = i;
n = 1;
idxs = 1:length(C);

while n>nn
    nn = n;
    back = idxs(ismember(C,customers));
    customers = sort([back C(customers) i]);
    customers = customers([true diff(customers)>0]);
    n = length(customers);
end
