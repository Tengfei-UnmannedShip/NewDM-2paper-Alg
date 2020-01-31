function feasible=ObstacleInDomain(map,x,y,extent)
%extent为船舶领域的大小

[m,n]=size(map);
if (y-extent<=0||y+extent>=n||x-extent<=0||x+extent>=m)%length(map(1,:))
    feasible = 0;
else
    range=map((x-extent):(x+extent),(y-extent):(y+extent));
    [r,c]=size(range);
    total=0;
    for ii=1:r
        for jj=1:c
            if range(ii,jj)==1
                total=total+1;
            end
        end
    end
    %     total=sum(sum(range));
    if(total>0)
        feasible = 0;
    else
        feasible = 1;
    end
end
end