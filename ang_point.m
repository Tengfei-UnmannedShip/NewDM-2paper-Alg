function Next_point = ang_point(x0,y0,ang,step_length)
% function goal_point = Goal_point(x0,y0,ang,range)����ú���
% Goal_point��ȫ�ֵ�ͼ�ϵĴ��ұ��ش���Ŀ���
% ang_point���Ա���Ϊ���ĺ�����һ�������ĵ�
% ˼·���ѱ�������ת�����е㣬���������ת����ȥ

point_temp= Goal_point(0,0,ang,step_length);

x=point_temp(1)+x0;
y=point_temp(2)+y0;

Next_point=[x,y];

end

