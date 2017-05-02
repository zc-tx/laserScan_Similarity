function [  ] = showImages( images )
%SAVEIMAGES Summary of this function goes here
%   Detailed explanation goes here

    len = length(images);
    
    for i = 1 : 1 : len
       image(images{i}*64);
       pause(1);
    end
    

end

