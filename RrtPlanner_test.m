close all;
tic
map0=PRM_map;
map0(map0<40)=0;
map0(map0>=40)=1;
map0=flipud(map0);

img = ~map0;
[h,w]=size(img);
imshow(img);
hold on;
startLocation(1,2) = round((Boat(2).pos(1)+MapSize(1)*1852)/Res)+1;
startLocation(1,1) = round((Boat(2).pos(2)+MapSize(2)*1852)/Res)+1;
endLocation(1,2) = round((Boat(2).goal(1)+MapSize(1)*1852)/Res)+1;
endLocation(1,1) = round((Boat(2).goal(2)+MapSize(1)*1852)/Res)+1;

p=startLocation;                %ѡȡ��ʼ�����λ��
p=[p;endLocation];
plot(p(:,1),p(:,2),'r.');

pc = p(1,:);                %����ڵ��б�
step = 20;                  %�����չ����
parent = 1;                 %���нڵ�ǰ������ʼ�ڵ�ǰ��Ϊ�Լ�

while norm(pc(end,:)-p(2,:))>step           %��������������ڵ�һ������ֹͣ
    
    if rand()<0.3                           %��30%�������������70%���ʳ��Ž���λ������
        nextp = [rand()*h rand()*w];
    else
        nextp = p(2,:);
    end
    
    diff = repmat(nextp,length(pc(:,1)),1)-pc;          %����ڵ�����������ڵ����
    [~,ind] = min(sqrt(diff(:,1).^2+diff(:,2).^2));     %�ҵ�����������ڵ���С�Ľڵ����ڵ�
    
    direct = atan2(nextp(1)-pc(ind,1),nextp(2)-pc(ind,2));
    sin_dir = sin(direct);
    cos_dir = cos(direct);
    newp = pc(ind,:) + step*[sin_dir cos_dir];          %���Ŵ������ڵ㷽����չ�ڵ���
    isobs = check_obs(img,newp,pc(ind,:));              %�жϸ�·���Ƿ����ϰ���
   
    if isobs==1                                         %���ϰ�����������
        continue;
    end
     
    diff = repmat(newp,length(pc(:,1)),1)-pc;           %�жϸ�·���Ƿ����������������������������������
    if min(sqrt(diff(:,1).^2+diff(:,2).^2))<sqrt(step)
        continue;
    end
    
    pc=[pc;newp];                                       %���½ڵ����ڵ���
    parent = [parent;ind];                              %�����½ڵ��ǰ��
    
    line([pc(ind,1) pc(parent(ind),1)],[pc(ind,2) pc(parent(ind),2)]);
end

line([pc(ind,1) p(2,1)],[pc(ind,2) p(2,2)],'color','r');
ind = length(pc);
while ind~=1
    ind = parent(ind);                                  %����������ǰ�ڵ�ĸ��ڵ�
    line([pc(ind,1) pc(parent(ind),1)],[pc(ind,2) pc(parent(ind),2)],'color','r');
end
toc