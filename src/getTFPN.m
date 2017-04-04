function [ TP, FP, TN, FN ] = getTFPN( SM01True, SM01Input )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    
%     lenTrue = length(IndexTrue);
%     lenInput = length(IndexInput);
%     
%     for i = 1 : 1 : lenTrue
%        pos = IndexTrue(i, :);
%        
%         
%     end
    
    len = length(SM01True);
    TP = 0;
    FP = 0;
    TN = 0;
    FN = 0;

    for i = 1 : 1 : len
        for j = 1 : 1 : i-1
            if SM01True(i, j) == 0
               if SM01Input(i, j) == 0
                    TP = TP + 1;
               else
                    FN = FN + 1;
               end    
            else
               if SM01Input(i, j) == 1
                   TN = TN + 1;
               else
                   FP = FP + 1;
               end
            end
            
            
        end
    end
    
end

