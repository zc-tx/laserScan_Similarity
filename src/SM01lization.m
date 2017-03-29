function SM01 = SM01lization( SM, threshold, judge)
%SM01LIZATION Summary of this function goes here
%   Detailed explanation goes here
    l = length(SM);    
    SM01 = zeros(l, l);
    
    %judge:0  ---> if > threshold  ---> loop closed
    %OR NOT
    
    for i = 1 : 1 : l
        for j = 1 : 1 : l
            
            if j == i
                SM01(j, i) = 0;
                continue;
            end
            
            if judge == 0
                if SM(i, j) > threshold
                    SM01(j, i) = 0;
                    SM01(i, j) = 0;
                else
                    SM01(j, i) = 1;
                    SM01(j, i) = 1;
                end
            else
                if SM(i, j) < threshold
                    SM01(j, i) = 0;
                    SM01(i, j) = 0;
                else
                    SM01(j, i) = 1;
                    SM01(j, i) = 1;
                end
            end
            
            
        end
    end
    
    imshow(SM01);
end

