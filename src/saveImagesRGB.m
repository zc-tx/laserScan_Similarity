function [  ] = saveImagesRGB( set, imagesRange, imagesIntensity )
%SAVEIMAGESRGB Summary of this function goes here
%   Detailed explanation goes here
    
    len = length(imagesRange);
    RGB = zeros(64, 80, 3);
    dir = '/home/yh/images/';
    
    for i = 1 : 1 : len
        i
        for m = 1 : 1 : 64
           for n = 1 : 1 : 80
              RGB(m, n, 1) = uint8(imagesRange{i}(m, n)*255);
              RGB(m, n, 2) = uint8(imagesIntensity{i}(m, n)*255);
              RGB(m, n, 3) = uint8(0); 
           end
        end
        
        name = sprintf('%04d.jpg', i);
        setname = num2str(set);
        
        filename = [dir, setname, name];
        
        filename
        
        imwrite(RGB, filename);
        
    end

end

