function SM = cosSM( images )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    frames = length(images);
    SM = zeros(frames, frames);
    
    
    for i = 1 : 1 : frames
       for j = 1 : 1 : i
          
%            if j == i
%                 SM(j, i) = 0;
%            end
                      
           A = images{i};
           B = images{j};
           sumCos = 0;
           
           for k = 1 : 1 : length(A)
               C = A(k, :);
               D = B(k, :);
               sumCos = sumCos + dot(C, D) / (norm(C)*norm(D));    
           end
           
           cosSimity = sumCos / length(A);
           
           SM(j, i) = cosSimity;
           SM(i, j) = cosSimity;
           
       end        
    end
    


end

