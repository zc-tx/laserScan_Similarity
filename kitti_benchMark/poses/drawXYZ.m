function drawXYZ( pose )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% plot every 10'th pose
figure; hold on; axis equal;
l = 3; % coordinate axis length
A = [0 0 0 1; l 0 0 1; 0 0 0 1; 0 l 0 1; 0 0 0 1; 0 0 l 1]';
for i=1:1:length(pose)
    p = pose{i};
    plot3(p(1,4),p(2,4),p(3,4)+i*0.1,'k.')  % show the loop
%   B = pose{i}*A;
%   plot3(B(1,1:2),B(2,1:2),B(3,1:2),'-r','LineWidth',2); % x: red
%   plot3(B(1,3:4),B(2,3:4),B(3,3:4),'-g','LineWidth',2); % y: green
%   plot3(B(1,5:6),B(2,5:6),B(3,5:6),'-b','LineWidth',2); % z: blue
end
xlabel('x');
ylabel('y');
zlabel('z');

end

