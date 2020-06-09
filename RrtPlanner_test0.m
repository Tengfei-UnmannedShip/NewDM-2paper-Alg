% RRT算法参数测试，验证从实际坐标转换到im坐标的公式，以及合适的船舶风险范围阈值

close all
tic

%% 需要用到的数据
% load RRT_testdata
pos=[9828.99476152239,8240.07928911434];
Theta=[-7973.54112569458,7827.44161566514];
MapSize=[8,8];
Res=18.5200000000000;
speed=10.2888888888889;
t1=toc;
%% 各点从实际坐标转换到im坐标的公式
p(:,1)=round(abs((Theta(1)+MapSize(1)*1852)/Res))+1;
p(:,2)=round(abs((-Theta(2)+MapSize(2)*1852)/Res))+1;
start(1)=round(abs((pos(1)+MapSize(1)*1852)/Res))+1;
start(2)=round(abs((-pos(2)+MapSize(2)*1852)/Res))+1;

threshold=165;
map0=RRT_map;
map0(map0<threshold)=0;
map0(map0>=threshold)=1;
map0=flipud(map0);
img = ~map0;
figure
imshow(img);
axis on
hold on;
% plot(p(2:3,1),p(2:3,2),'r*');
plot(p(1,1),p(1,2),'r^');
plot(start(1),start(2),'ro');
t02=toc;
t2=t02-t1;
%% 用while函数递推的方法找到合适的threshold值，防止路径点在障碍物范围内
threshold=165;
while img(p(1),p(2))==0
    threshold=threshold+1
    map0=RRT_map;
    map0(map0<threshold)=0;
    map0(map0>=threshold)=1;
    map0=flipud(map0);
    img = ~map0;
end
disp(['最终 threshold=',num2str( threshold)]);
figure
imshow(img);
axis on
hold on;

plot(p(1,1),p(1,2),'r^');
plot(start(1),start(2),'ro');
t03=toc;
t3=t03-t02;
%% 全部的RRT算法测试
% threshold=threshold+1
map0=RRT_map;
map0(map0<threshold)=0;
map0(map0>=threshold)=1;
map0=flipud(map0);
img = ~map0;

[h,w]=size(img);
imshow(img);
hold on;

startLocation =start;
endLocation = p(1,:);

p=startLocation;                %选取起始与结束位置
p=[p;endLocation];
plot(p(:,1),p(:,2),'r.');

pc = p(1,:);                %随机节点列表
step = speed;                  %随机扩展步长
parent = 1;                 %所有节点前驱，初始节点前驱为自己

while norm(pc(end,:)-p(2,:))>step           %搜索到距离结束节点一定距离停止
    
    if rand()<0.3                           %按30%概率随机搜索，70%概率朝着结束位置搜索
        nextp = [rand()*h rand()*w];
    else
        nextp = p(2,:);
    end
    
    diff = repmat(nextp,length(pc(:,1)),1)-pc;          %计算节点树与待搜索节点距离
    [~,ind] = min(sqrt(diff(:,1).^2+diff(:,2).^2));     %找到距离带搜索节点最小的节点树节点
    
    direct = atan2(nextp(1)-pc(ind,1),nextp(2)-pc(ind,2));
    sin_dir = sin(direct);
    cos_dir = cos(direct);
    newp = pc(ind,:) + step*[sin_dir cos_dir];          %向着待搜索节点方向扩展节点树
    isobs = check_obs(img,newp,pc(ind,:));              %判断该路径是否有障碍物
    
    if isobs==1                                         %有障碍物重新搜索
        continue;
    end
    
    diff = repmat(newp,length(pc(:,1)),1)-pc;           %判断该路径是否已搜索过，如果已搜索过，则重新搜索
    if min(sqrt(diff(:,1).^2+diff(:,2).^2))<sqrt(step)
        continue;
    end
    
    pc=[pc;newp];                                       %将新节点加入节点树
    parent = [parent;ind];                              %设置新节点的前驱
    
    line([pc(ind,1) pc(parent(ind),1)],[pc(ind,2) pc(parent(ind),2)]);
end

line([pc(ind,1) p(2,1)],[pc(ind,2) p(2,2)],'color','r');
ind = length(pc);
while ind~=1
    ind = parent(ind);                                  %不断搜索当前节点的父节点
    line([pc(ind,1) pc(parent(ind),1)],[pc(ind,2) pc(parent(ind),2)],'color','r');
end
t04=toc;
t4=t04-t03;


