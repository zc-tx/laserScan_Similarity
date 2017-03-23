function images = getImageCells( Bins )
%GETIMAGECELLS Summary of this function goes here
%   Detailed explanation goes here
    l = length(Bins) / 64;
    images= cell(1, l);
    
    for i = 1 : 1 : l
        image = Bins( (i-1)*64 + 1 : i*64, : );
        images{1, i} = image; 
    end
    

end

