function WP= WayPoint(pos_os,course_ts,pos_ts,TSlength,changeLabel)
%% 2�Ҵ���TS��ͷ��β��·������㣬�����ɱ�Ͳ��ɱ�·����
% changeLabel=0,·����̶�
% changeLabel=1,·���㲻�̶�
% OS����ÿһ��TS���������Ŀ��·���㣬���Ϊ��ͷ+��β
% ������CPA���㵫�Ƕ���һ������ʱ�䣬����Jinfen���㷨�м���ÿһ�׶ε�CPA
% ��������Ŀ�괬���ٶ���(�ٶ�ֵ+���н�)�ļ�������ʽת��Ϊ(Vx,Vy)��ֱ��������ʽ
time=1000;

dis=norm(pos_os-pos_ts);
if  changeLabel==0
    a1=2;  %����ͷ2����������β1.5���������
    a2=1.5;
else
    if dis<200
        a1=200/TSlength;  %���������ʱ�򰴴�ͷ200�ף���β100�����
        a2=100/TSlength;
    elseif dis>200 && dis<500
        a1=500/TSlength;  %���������ʱ�򰴴�ͷ500�ף���β300�����
        a2=300/TSlength;
    elseif dis>500 && dis<1000
        a1=1;  %����ͷ1����������β0.75���������
        a2=0.75;
    else
        a1=2;  %����ͷ2����������β1.5���������
        a2=1.5;
    end
end

x0=pos_ts(1);
y0=pos_ts(2);
x1=a1*TSlength*sind(course_ts)+x0;       %��ͷĿ���x���꣬��ͷa1��������
y1=a1*TSlength*cosd(course_ts)+y0;       %��ͷĿ���y���꣬��ͷa1��������
x2=x0-a2*TSlength*sind(course_ts);       %��βĿ���x���꣬��βa2��������
y2=y0-a2*TSlength*cosd(course_ts);       %��βĿ���y���꣬��βa2��������

WP=[x1 y1 x2 y2];


end

