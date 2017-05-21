function [  ] = getTestLabel( set, int )
%GETTESTLABEL Summary of this function goes here
%   Detailed explanation goes here

    dir = '/home/yh/kitti_test/imagesName/';
    setName = num2str(set);
    
    fileName = [dir, setName, 'testLabel.txt'];
    fid = fopen(fileName, 'w+');

    for i = 1 : 1 : int
        for j = 1 : 1 : i-1
            name1 = sprintf('%04d', i);
            name2 = sprintf('%04d', j);
            imageName1 = [setName, name1, '.jpg   ']; 
            imageName2 = [setName, name2, '.jpg']; 
            fprintf(fid, '%s', imageName1);
            fprintf(fid, '%s\n', imageName2);  
        end
    end
    
end

