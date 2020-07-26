
path = path2;
n = size(path, 1);
index1 = 1;
index2 = 2;
dis1 = 0;
dis = 10;
trajectory = [];
while(1)
    [point,theta,index1, index2, dis1, dis2] = GetNextPoint(path, index1, index2, dis1, dis);
    trajectory = [trajectory; point];
    d = sqrt(sum((point - path(n, :)).^2));
    if d < dis
        break;
    end
end

figure
plot(path(:, 1), path(:, 2), 'r.')
hold on 
plot(trajectory(:, 1), trajectory(:, 2), 'b.')
axis equal