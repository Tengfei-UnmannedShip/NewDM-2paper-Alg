function feasible=PointsCanReach(r,Rmin,angle,direction)

feasible=0;
if Rmin<r/2
    feasible=1;
else
    apha = pi/2 - acos(r/(2*Rmin));
    IncludedAngle = abs(angle-direction);
    if IncludedAngle>pi
        IncludedAngle = 2*pi-IncludedAngle;
    end
    if IncludedAngle < apha
        feasible = 1;
    end
end
end

