function  WayPoint = WP_2ship1(v_ts,course_ts,pos_ts,TSlength)
%% 2�Ҵ���TS��ͷ��β��·������㣬����λ���ڵ�ǰλ��
% �����ɱ�Ͳ��ɱ�·����
% changeLabel=0,·����̶�
% changeLabel=1,·���㲻�̶�
% OS����ÿһ��TS���������Ŀ��·���㣬���Ϊ��ͷ+��β

    a1=2.5;  %����ͷ2����������β1.5���������
    a2=2;

x0=pos_ts(1);
y0=pos_ts(2);
x1=a1*TSlength*v_ts*sind(course_ts)+x0;       %��ͷĿ���x���꣬��ͷa1��������
y1=a1*TSlength*v_ts*cosd(course_ts)+y0;       %��ͷĿ���y���꣬��ͷa1��������
x2=x0-a2*TSlength*v_ts*sind(course_ts);       %��βĿ���x���꣬��βa2��������
y2=y0-a2*TSlength*v_ts*cosd(course_ts);       %��βĿ���y���꣬��βa2��������

WayPoint=[x1 y1 x2 y2];

end


