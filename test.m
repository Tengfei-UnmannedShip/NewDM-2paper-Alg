if isempty(Real_CAL)
    Boat(OS).WayPoint=Boat(OS).goal;  %û��·�����ʱ��ֱ�����յ���ΪĿ���
    disp([num2str(OS),'��Ŀ���Ϊ�յ�']);
else
    CAL_row=[];
    switch WP_Num
        case 1
            CAL_row=Real_CAL+1;
            disp([num2str(OS),'����1�ҷ��մ�������Ϊ',num2str(Real_CAL)]);
        case 2
            CAL_row=2^Real_CAL(1)+Real_CAL(2)+1;
            disp([num2str(OS),'����2�ҷ��մ�������Ϊ',num2str(Real_CAL)]);
        case 3
            CAL_row=4^Real_CAL(1)+2^Real_CAL(2)+Real_CAL(3)+1;
            disp([num2str(OS),'����3�ҷ��մ�������Ϊ',num2str(Real_CAL)]);
    end
    % �жϵ�ǰ��·�����Ƿ��ڵ�ͼ��Χ�ڣ����ڵĻ�����·����Ϊԭ·�����뱾��λ��������߽�Ľ���
    WP_test0=Boat(OS).WayPoint_temp(CAL_row,:);
    Boat(OS).WayPoint=WP_regiontest(WP_test0,MapSize,Boat(OS).pos);
end

