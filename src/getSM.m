function SM = getSM( Bin )
%GETSM Summary of this function goes here
%   Detailed explanation goes here
    % Just to draw Similarity Matrix

    frames = length(Bin);
    sections = size(Bin, 2);
    SM = zeros(frames, frames);

    for i = 1 : 1 : length(Bin)
        for j = 1 : 1 : i

            if j == i
                SM(j, i) = 0; % 0:similar place 1:non-similar place
            end

            EMD = 0;
            for m = 1 : 1 : sections
                for k = 1 : 1 : m
                   delta = abs(Bin(i, k) - Bin(j, k));
                   EMD = EMD + delta;
                end
            end

             EMD = EMD / sections;


%             if EMD < 0.100
%                 SM(j, i) = 0;
%                 SM(i, j) = 0;
%             else
%                 SM(j, i) = 1;
%                 SM(i, j) = 1;
%             end   
            SM(i, j) = EMD;
            SM(j, i) = EMD;

        end
    end
end

