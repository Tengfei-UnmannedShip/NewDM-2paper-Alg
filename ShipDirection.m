function Dir= ShipDirection(frontPosition_x,frontPosition_y,currentPosition_x,currentPosition_y)
    if (currentPosition_y>=frontPosition_y)
        Dir=atan((currentPosition_x-frontPosition_x)/(currentPosition_y-frontPosition_y));
    else
        Dir=pi+atan((currentPosition_x-frontPosition_x)/(currentPosition_y-frontPosition_y));
    end
end