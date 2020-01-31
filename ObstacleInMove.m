function feasible=ObstacleInMove(map,position_x,position_y,nextposition_x,nextposition_y,HalfofShip)
% 用于计算移动？
total=0;
if (nextposition_y-HalfofShip<=0||nextposition_y+HalfofShip>=length(map(1,:))||nextposition_x-HalfofShip<=0||nextposition_x+HalfofShip>=length(map(:,1))) ...
        ||(position_y-HalfofShip<=0||position_y+HalfofShip>=length(map(1,:))||position_x-HalfofShip<=0||position_x+HalfofShip>=length(map(:,1)))
    feasible = 0;
else
    if (nextposition_x > position_x)
        num = ceil(sqrt((position_x-nextposition_x)^2+(position_y-nextposition_y)^2)/(HalfofShip*2));
        for ii = 1:num
            x = floor(position_x+ii*(nextposition_x-position_x)/num);
            y = floor(position_y+ii*(nextposition_y-position_y)/num);
            if ObstacleInDomain(map,x,y,HalfofShip)==0
                total=total+1;
            end
        end
    else
        num = ceil(sqrt((position_y-nextposition_y)^2+(position_x-nextposition_x)^2)/(HalfofShip*2));
        for ii = 1:num
            x = floor(nextposition_x+ii*(position_x-nextposition_x)/num);
            y = floor(nextposition_y+ii*(position_y-nextposition_y)/num);
            if ObstacleInDomain(map,x,y,HalfofShip)==0
                total=total+1;
            end
        end
    end
    if(total>0)
        feasible = 0;
    else
        feasible = 1;
    end
end
% imshow(map);
end