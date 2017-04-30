function [ trueIndex, falseIndex ] = getTFIndex( SM, unloopSelect )
%GETTFINDEX Summary of this function goes here
%   Detailed explanation goes here
    
    len = length(SM);
    unloop = 0;
    loopCount = 0;
    unloopCount = 0;

    for i = 1 : 1 : len
        for j = 1 : 1 : i-1
           
            if SM(i, j) == 0  %loop
                loopCount = loopCount + 1;
                trueIndex(loopCount, :) = [i, j];
            end
            
            if SM(i, j) == 1
                unloop = unloop + 1;
            end
            
            if mod(unloop, unloopSelect) == 0
               unloop = 0;
               unloopCount = unloopCount + 1;
               falseIndex(unloopCount, :) = [i, j];
            end
            
        end
        
    end


end

