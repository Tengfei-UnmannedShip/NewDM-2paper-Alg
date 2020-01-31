function bool=alreadyexist(point,list)
    bool=0;
    for ii=1:length(list)
        if point.y == list(ii).y && point.x==list(ii).x
            bool = 1;
        end
    end
end