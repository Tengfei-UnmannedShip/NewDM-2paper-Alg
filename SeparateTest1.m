load test-2.mat
%% ��ͼ����
MapSize=[8,8];
Res=18.52*2;  %Resolution��ͼ�ķֱ���
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
[m,n]=size(X);

%% ÿ�Ҵ�·���滮��W1��4Ϊÿһ�Ҵ���ǰ��FMM�����ͼ��start��end�ֱ�Ϊ�����յ�
FM_map=W4+0.0001;
start_point=start4;
end_point=end4;
%%ע�ͣ������ԣ�����ֻ��W1,start1,end1����һ�Ҵ�ʱ�����������У�
% ����2��4�Ҵ���Ȼ���ǰ���ͬ���ķ�ʽ���ɵģ������޷�����
% ������鷢���������ڣ���2��4ʱ��FMM�����е�12��[D,~] = perform_fast_marching(W, start_points, options); 
% ���ɵ�ͼW������ʼ���ֵΪ0������ֵȫ��Inf����֪��Ϊʲô����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
[Mtotal, paths] = FMM(FM_map,start_point',end_point');
toc

FinalMap=Mtotal;
FMpath = paths{:};
path0=fliplr(FMpath');  %FMM�滮������·�����ҵ�����XY�Ƿ���
posData = zeros(size(path0));
posData(:,1)=path0(:,1)*Res-MapSize(1)*1852;   %դ������ת����ʵ������
posData(:,2)=path0(:,2)*Res-MapSize(2)*1852;

start_pos(1)=start_point(2)*Res-MapSize(1)*1852; %FMM�滮������·�����ҵ�����XY�Ƿ���
start_pos(2)=start_point(1)*Res-MapSize(2)*1852;

end_pos(1)=end_point(2)*Res-MapSize(1)*1852; %FMM�滮������·�����ҵ�����XY�Ƿ���
end_pos(2)=end_point(1)*Res-MapSize(2)*1852;

figure
contourf(X,Y,FM_map);  %�������ɫ�ĵȸ���ͼ
hold on
plot(start_pos(1,1),start_pos(1,2),'ro');
hold on
plot(end_pos(1),end_pos(2),'r*');
hold on
plot(posData(:, 1), posData(:, 2), 'r-');

axis([-MapSize(1)*1852 MapSize(1)*1852 -MapSize(2)*1852 MapSize(2)*1852])
set(gca,'XTick',MapSize(1)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'XTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
set(gca,'YTick',MapSize(2)*1852*[-1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1]);
set(gca,'YTickLabel',{'-8','-6','-4','-2','0','2','4','6','8'},'Fontname','Times New Roman');
grid on;
xlabel('\it n miles', 'Fontname', 'Times New Roman');
ylabel('\it n miles', 'Fontname', 'Times New Roman');

%% �����ͼ�Ա�
figure
subplot(2,2,1)
contourf(X,Y,W1*100,10);
title('1�Ŵ�����ͼ')
subplot(2,2,2)
contourf(X,Y,W2*100,10);
title('2�Ŵ�����ͼ')
subplot(2,2,3)
contourf(X,Y,W3*100,10);
title('3�Ŵ�����ͼ')
subplot(2,2,4)
contourf(X,Y,W4*100,10);
title('4�Ŵ�����ͼ')