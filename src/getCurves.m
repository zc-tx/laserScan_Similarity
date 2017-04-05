function [PRCurve, ROCCurve] = getCurves( SM01True, SMInput )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    count = 1;
    for tao = 0 : 0.01 : 1 
        tic
        SM01Input = SM01lization(SMInput, tao, 0);
        
        [ TP, FP, TN, FN ] = getTFPN(SM01True, SM01Input);
        
        %Precision
        PRCurve(count, 1) = TP / (TP + FP);
        
        %Recall
        PRCurve(count, 2) = TP / (TP + FN);
        
        %TPR = Recall
        ROCCurve(count, 1) = TP / (TP + FN);
        
        %FPR
        ROCCurve(count, 2) = FP / (FP + TN);
               
        count = count + 1;
        toc
    end


end

