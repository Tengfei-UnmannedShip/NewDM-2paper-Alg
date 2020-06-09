function  RrtPath = RRTPlanner(RRT_map,startLocation,endLocation,speed)  
% 由RrtPlanner_test调试后改成了函数
% 输入%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% map0:直接输入风险图即可，程序中对风险图二值化
% startLocation,endLocation：起始点和终点的位置，注意位置数据的预处理：
% startLocation(1)=round(abs(( startpos(1)+MapSize(1)*1852)/Res))+1;
% startLocation(2)=round(abs((-startpos(2)+MapSize(2)*1852)/Res))+1;
% speed为每一步的步长
% threshold为地图中风险的阈值，为防止目标点在障碍物中从而无法计算，需要事先判断
%                当前的threshold是测试结果，为165，可能会有不同
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 输出%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RrtPath:RRT算法生成的路径
% parent：每一步的父节点
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 数据预处理，风险图的二值化
threshold=166;
map0=RRT_map;
map0(map0<threshold)=0;
map0(map0>=threshold)=1;
map0=flipud(map0);
img = ~map0;
[h,w]=size(img);

p=startLocation;                %选取起始与结束位置
p=[p;endLocation];

while img(endLocation(1,1),endLocation(1,2))==0 
    threshold=threshold+1;
    map0=RRT_map;
    map0(map0<threshold)=0;
    map0(map0>=threshold)=1;
    map0=flipud(map0);
    img = ~map0;
    if threshold>max(max(RRT_map))
       disp('报错：生成RRT地图时threshold过大');
       break
    end
end

RrtPath = p(1,:);                %随机节点列表
step = speed;                  %随机扩展步长
parent = 1;                 %所有节点前驱，初始节点前驱为自己

while norm(RrtPath(end,:)-p(2,:))>step           %搜索到距离结束节点一定距离停止
    
    if rand()<0.3                           %按30%概率随机搜索，70%概率朝着结束位置搜索
        nextp = [rand()*h rand()*w];
    else
        nextp = p(2,:);
    end
    
    diff = repmat(nextp,length(RrtPath(:,1)),1)-RrtPath;          %计算节点树与待搜索节点距离
    [~,ind] = min(sqrt(diff(:,1).^2+diff(:,2).^2));     %找到距离带搜索节点最小的节点树节点
    
    direct = atan2(nextp(1)-RrtPath(ind,1),nextp(2)-RrtPath(ind,2));
    sin_dir = sin(direct);
    cos_dir = cos(direct);
    newp = RrtPath(ind,:) + step*[sin_dir cos_dir];          %向着待搜索节点方向扩展节点树
    isobs = check_obs(img,newp,RrtPath(ind,:));              %判断该路径是否有障碍物
    
    if isobs==1                                         %有障碍物重新搜索
        continue;
    end
    
    diff = repmat(newp,length(RrtPath(:,1)),1)-RrtPath;           %判断该路径是否已搜索过，如果已搜索过，则重新搜索
    if min(sqrt(diff(:,1).^2+diff(:,2).^2))<sqrt(step)
        continue;
    end
    
    RrtPath=[RrtPath;newp];                                       %将新节点加入节点树
    parent = [parent;ind];                              %设置新节点的前驱
    if size(RrtPath,1)>1000
        break       %总是出现RrtPath太长的情况，因此，设置大于1000时，停止计算，本次仿真结束
    end
end




