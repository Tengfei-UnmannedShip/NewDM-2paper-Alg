function isobs = check_obs(img,p1,p2)  
[h, w]=size(img);
d = norm(p1-p2);
direct = atan2(p1(1)-p2(1),p1(2)-p2(2));
sin_dir = sin(direct);
cos_dir = cos(direct);
for r=0:d
    p = floor(p2 + r*[sin_dir cos_dir]);
    
    y = p(2);
    x = p(1);
    if y>=1 && y<=h && x>=1 && x<=w      
        if img(y,x) ==0
            isobs = 1;
            return;
        end
    end
end
isobs = 0;
end