


r = 0:10;
theta = 15:1:345;
Z = ones(numel(r),numel(theta));
% Z(r<7,:) = 2;
Z(r<3,:) = 3;
X = r(:)*cosd(theta);
Y = r(:)*sind(theta);
surf(X,Y,Z)