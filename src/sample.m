function imageSample = sample( image )
%SAMPLE Summary of this function goes here
%   Detailed explanation goes here

    l = length(image);
    count = 1;
    for i=1 : 10 : l
       imageSample{count} = image{i};     
       count = count + 1;
    end

end

