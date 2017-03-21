l = length(BinCopy);
count=1;
for i = 1 : 10 : l
    Bin(count, :) = BinCopy(i, :); 
    count = count + 1;
end