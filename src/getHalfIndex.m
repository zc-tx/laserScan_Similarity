function loopIndex = getHalfIndex ( SM01 )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    len = length(SM01);
    count = 1;
    
    for i = 1 : 1 : len
        for j = 1 : 1 : i-1
           if SM01(i, j) == 0
              loopIndex(count, 1) = i;
              loopIndex(count, 2) = j;
              count = count + 1;
           end            
        end
    end

end

