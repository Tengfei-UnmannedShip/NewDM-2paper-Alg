function  RrtPath = RRTPlanner(RRT_map,startLocation,endLocation,speed)  
% ��RrtPlanner_test���Ժ�ĳ��˺���
% ����%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% map0:ֱ���������ͼ���ɣ������жԷ���ͼ��ֵ��
% startLocation,endLocation����ʼ����յ��λ�ã�ע��λ�����ݵ�Ԥ����
% startLocation(1)=round(abs(( startpos(1)+MapSize(1)*1852)/Res))+1;
% startLocation(2)=round(abs((-startpos(2)+MapSize(2)*1852)/Res))+1;
% speedΪÿһ���Ĳ���
% thresholdΪ��ͼ�з��յ���ֵ��Ϊ��ֹĿ������ϰ����дӶ��޷����㣬��Ҫ�����ж�
%                ��ǰ��threshold�ǲ��Խ����Ϊ165�����ܻ��в�ͬ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ���%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RrtPath:RRT�㷨���ɵ�·��
% parent��ÿһ���ĸ��ڵ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ����Ԥ��������ͼ�Ķ�ֵ��
threshold=166;
map0=RRT_map;
map0(map0<threshold)=0;
map0(map0>=threshold)=1;
map0=flipud(map0);
img = ~map0;
[h,w]=size(img);

p=startLocation;                %ѡȡ��ʼ�����λ��
p=[p;endLocation];

while img(endLocation(1,1),endLocation(1,2))==0 
    threshold=threshold+1;
    map0=RRT_map;
    map0(map0<threshold)=0;
    map0(map0>=threshold)=1;
    map0=flipud(map0);
    img = ~map0;
    if threshold>max(max(RRT_map))
       disp('��������RRT��ͼʱthreshold����');
       break
    end
end

RrtPath = p(1,:);                %����ڵ��б�
step = speed;                  %�����չ����
parent = 1;                 %���нڵ�ǰ������ʼ�ڵ�ǰ��Ϊ�Լ�

while norm(RrtPath(end,:)-p(2,:))>step           %��������������ڵ�һ������ֹͣ
    
    if rand()<0.3                           %��30%�������������70%���ʳ��Ž���λ������
        nextp = [rand()*h rand()*w];
    else
        nextp = p(2,:);
    end
    
    diff = repmat(nextp,length(RrtPath(:,1)),1)-RrtPath;          %����ڵ�����������ڵ����
    [~,ind] = min(sqrt(diff(:,1).^2+diff(:,2).^2));     %�ҵ�����������ڵ���С�Ľڵ����ڵ�
    
    direct = atan2(nextp(1)-RrtPath(ind,1),nextp(2)-RrtPath(ind,2));
    sin_dir = sin(direct);
    cos_dir = cos(direct);
    newp = RrtPath(ind,:) + step*[sin_dir cos_dir];          %���Ŵ������ڵ㷽����չ�ڵ���
    isobs = check_obs(img,newp,RrtPath(ind,:));              %�жϸ�·���Ƿ����ϰ���
    
    if isobs==1                                         %���ϰ�����������
        continue;
    end
    
    diff = repmat(newp,length(RrtPath(:,1)),1)-RrtPath;           %�жϸ�·���Ƿ����������������������������������
    if min(sqrt(diff(:,1).^2+diff(:,2).^2))<sqrt(step)
        continue;
    end
    
    RrtPath=[RrtPath;newp];                                       %���½ڵ����ڵ���
    parent = [parent;ind];                              %�����½ڵ��ǰ��
    if size(RrtPath,1)>1000
        break       %���ǳ���RrtPath̫�����������ˣ����ô���1000ʱ��ֹͣ���㣬���η������
    end
end




