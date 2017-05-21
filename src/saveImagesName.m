function [  ] = saveImagesName( set, trueIndex, falseIndex )
%SAVEIMAGESNAME Summary of this function goes here
%   Detailed explanation goes here
    
    dir = '/home/yh/kitti_test/imagesName/';
    setName = num2str(set);
    
    fileNameTrue = [dir, setName, 'True.txt'];
    fileNameFalse = [dir, setName, 'False.txt'];
    
    fidTrue = fopen(fileNameTrue, 'w+');
    fidFalse = fopen(fileNameFalse, 'w+');
    
    lenTrue = length(trueIndex);
    lenFalse = length(falseIndex);
    
    for i = 1 : 1 : lenTrue
        name1 = sprintf('%04d', trueIndex(i, 1));
        name2 = sprintf('%04d', trueIndex(i, 2));
        imageName1 = [setName, name1, '.jpg   ']; 
        imageName2 = [setName, name2, '.jpg']; 
        fprintf(fidTrue, '%s', imageName1);
        fprintf(fidTrue, '%s\n', imageName2);       
    end
    
    for j = 1 : 1 : lenFalse
        name1 = sprintf('%04d', falseIndex(i, 1));
        name2 = sprintf('%04d', falseIndex(i, 2));
        imageName1 = [setName, name1, '.jpg   ']; 
        imageName2 = [setName, name2, '.jpg']; 
        fprintf(fidFalse, '%s', imageName1);
        fprintf(fidFalse, '%s\n', imageName2);   
    end
    
    


end

