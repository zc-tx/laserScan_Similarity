function test = getXYZ( input )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    test = cell(1, length(input));
    for i = 1 : 1 : length(input)
        test{i} = [input(i, 9), input(i, 10), input(i, 11), input(i, 12); % FUCK!!!  % WRONG?
                   input(i, 1), input(i, 2), input(i, 3), input(i, 4);
                   input(i, 5), input(i, 6), input(i, 7), input(i, 8);
                   0, 0, 0, 1];
    end
end

