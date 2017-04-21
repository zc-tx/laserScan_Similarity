function SM = absSM( images )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    frames = length(images);
    SM = zeros(frames, frames);
    
    for i = 1 : 1 : frames
       i
       tic
       for j = 1 : 1 : i
           
           A = images{i}; 
           B = images{j};
           sumAbs = 0;
           
           for k = 1 : 1 : size(A, 1)
               for m = 1 : 1: size(A, 2)
                    sumAbs = sumAbs + abs(A(k, m) - B(k, m));
               end
           end
           
           absSimity = sumAbs / (size(A, 1) * size(A, 2));
           
           SM(j, i) = absSimity;
           SM(i, j) = absSimity;
           
       end        
       toc
    end
    

end

