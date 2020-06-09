% RRT�㷨�������ԣ���֤��ʵ������ת����im����Ĺ�ʽ���Լ����ʵĴ������շ�Χ��ֵ

close all
tic

%% ��Ҫ�õ�������
% load RRT_testdata
pos=[9828.99476152239,8240.07928911434];
Theta=[-7973.54112569458,7827.44161566514];
MapSize=[8,8];
Res=18.5200000000000;
speed=10.2888888888889;
t1=toc;
%% �����ʵ������ת����im����Ĺ�ʽ
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
%% ��while�������Ƶķ����ҵ����ʵ�thresholdֵ����ֹ·�������ϰ��ﷶΧ��
threshold=165;
while img(p(1),p(2))==0
    threshold=threshold+1
    map0=RRT_map;
    map0(map0<threshold)=0;
    map0(map0>=threshold)=1;
    map0=flipud(map0);
    img = ~map0;
end
disp(['���� threshold=',num2str( threshold)]);
figure
imshow(img);
axis on
hold on;

plot(p(1,1),p(1,2),'r^');
plot(start(1),start(2),'ro');
t03=toc;
t3=t03-t02;
%% ȫ����RRT�㷨����
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

p=startLocation;                %ѡȡ��ʼ�����λ��
p=[p;endLocation];
plot(p(:,1),p(:,2),'r.');

pc = p(1,:);                %����ڵ��б�
step = speed;                  %�����չ����
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
t04=toc;
t4=t04-t03;


