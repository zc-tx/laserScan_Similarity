function [ SMEMD, SMCos, SMAbs ] = SMCollection( images )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    frames = length(images);
    SMEMD = zeros(frames, frames);
    SMCos = zeros(frames, frames);
    SMAbs = zeros(frames, frames);
    sections = 80;

    for i = 1 : 1 : frames
       i
       tic
       for j = 1 : 1 : i
           
           A = images{i};
           B = images{j};
           sumEMD = 0;
           sumCos = 0;
           sumAbs = 0;

           for k = 1 : 1 : size(A,1)
                C = A(k, :);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
                D = B(k, :);

                %EMD compute
                EMD = 0;
                for m = 1 : 1 : sections
                    for n = 1 : 1 : m
                        delta = abs(C(1, n) - D(1, n));
                        EMD = EMD + delta;
                    end
                end
                EMD = EMD /sections;
                sumEMD = sumEMD + EMD;    

                %Cos compute
                sumCos = sumCos + dot(C, D) / (norm(C)*norm(D)); 

                %Abs compute
                for m = 1 : 1: size(A, 2)
                    sumAbs = sumAbs + abs(A(k, m) - B(k, m));
                end

           end

           %EMD
           EMDSimity = sumEMD / size(A,1);
           SMEMD(j, i) = EMDSimity;
           SMEMD(i, j) = EMDSimity;

           %Cos
           cosSimity = sumCos / size(A,1);
           SMCos(j, i) = cosSimity;
           SMCos(i, j) = cosSimity;

           %Abs
           absSimity = sumAbs / (size(A, 1) * size(A, 2));  
           SMAbs(j, i) = absSimity;
           SMAbs(i, j) = absSimity; 

       end
       toc
    end

end

