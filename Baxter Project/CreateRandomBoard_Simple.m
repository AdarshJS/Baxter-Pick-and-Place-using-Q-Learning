function [Board, color_3, color_2, color_1 ] = CreateRandomBoard_Simple()

colors = [1 2 3]; %1 for R, 2 for G and 3 for B
color_3 = 1; %Color_n represents the color that chosen to appear n times
color_2 = 2;
color_1 = 3;

Places = [1 2 3 4 5 6];
msize = numel(Places);
Places_3 = Places(randperm(msize, 3));
for i = 1:3
    Places(Places == Places_3(i))=[];
end

msize = numel(Places);
Places_2 = Places(randperm(msize, 2));

for i = 1:2
    Places(Places == Places_2(i))=[];
end

Places_1 = Places;

for i = 1:3
    Board(Places_3(i)) = color_3;
end
for i = 1:2
    Board(Places_2(i)) = color_2;
end
Board(Places_1) = color_1;

Board