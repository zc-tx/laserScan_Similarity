function SM = getTrueSM( testPose )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    
    len = length(testPose);
    SM = zeros(len, len);

    for i = 1 : 1 : len
        for j = 1 : 1 : i
            if i == j
                SM(i, j) = 0;
                SM(j, i) = 0;
                continue;
            end
            
            xyI = [testPose{i}(1, 4), testPose{i}(2, 4)];
            xyJ = [testPose{j}(1, 4), testPose{j}(2, 4)];
            
            dis = norm(xyI - xyJ);
            
            if dis < 3 && (i - j) > 30
                SM(i, j) = 0;
                SM(j, i) = 0;
            else
                SM(i, j) = 1;
                SM(j, i) = 1;
            end
        end
    end
        


end

