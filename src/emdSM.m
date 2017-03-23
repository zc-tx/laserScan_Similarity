function SM = emdSM( images )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    frames = length(images);
    SM = zeros(frames, frames);
    sections = 40;
    
    for i = 1 : 1 : frames
       for j = 1 : 1 : i
          
%            if j == i
%                 SM(j, i) = 0;
%            end
                      
           A = images{i};
           B = images{j};
           sumEMD = 0;
           
           for k = 1 : 1 : length(A)
                C = A(k, :);
                D = B(k, :);
                EMD = 0;
                
                for m = 1 : 1 : sections
                    for n = 1 : 1 : m
                        delta = abs(C(1, n) - D(1, n));
                        EMD = EMD + delta;
                    end
                end
                EMD = EMD /sections;
                
                sumEMD = sumEMD + EMD;    
           end
           
           EMDSimity = sumEMD / length(A);
           
           SM(j, i) = EMDSimity;
           SM(i, j) = EMDSimity;
           
       end        
    end
    


end

