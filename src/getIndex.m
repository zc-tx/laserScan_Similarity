function res = getIndex( SM )
%GETINDEX Summary of this function goes here
%   Detailed explanation goes here

    loop = SM ==0;
    idx = find(loop);
    l = length(SM);
    res = [mod(idx,l) ceil(idx/l)];
    res(res(:,1)==0,1) = l;

end

