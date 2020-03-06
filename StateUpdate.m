function Boat = StateUpdate(Boat0,Boat_Num,t,Res)
% ÿ��ʱ�̵ĳ���״̬���£��ֳɲ�ͬ�����
Boat=Boat0;

for i=1:1:Boat_Num
    if Boat(i).FM_lable~=0 && Boat(i).reach==1 %���Ѿ���ʼ���ߵ�û�е��յ�
        %����FMM�������ص㣬��Щ��֮�䲽���ر𳤣������Ҫ�ֳ��������
        row=Boat(i).Current_row;
        deltaPos=Boat(i).path(row+1,:)-Boat(i).pos;
        LastPos=Boat(i).pos;
        if  norm(deltaPos)>Boat(i).speed
            %�������ڵ�λʱ�䴬���ߵ�·�̵�ʱ����Ҫ���Ź滮֮�������֮����߶��ҵ�
            disp([num2str(t),'ʱ��',num2str(i),'�Ŵ��������ڵ�λʱ��·��']);
            x_temp=Boat(i).pos(1);
            y_temp=Boat(i).pos(2);
            pos_temp0=[x_temp,y_temp];
            step_temp=0;
            while step_temp<Boat(i).speed
                x_l=Boat(i).path(row:row+1,1);
                y_l=Boat(i).path(row:row+1,2);
                %��ȡ��ֵ�����ҵ�
                if x_l(1)==x_l(2)  %һ����ֱ��
                    x_temp=x_temp;
                    y_temp=y_temp+1;   %ÿ����1��
                elseif y_l(1)==y_l(2)  %һ��ˮƽ��
                    x_temp=x_temp+1;     %ÿ����1��
                    y_temp=y_temp;
                else   %һ�������б��
                    x_temp=x_temp+1;    %ÿ��x��1��
                    y_temp=interp1(x_l,y_l,x_temp,'spline');
                end
                pos_temp=[x_temp,y_temp];
                delta_pos0=pos_temp-pos_temp0;
                delta_pos=norm(delta_pos0);
                step_temp=step_temp+delta_pos;
                pos_temp0=pos_temp;
            end
            Boat(i).pos=pos_temp;
        else    %����С�ڵ�λʱ��·�̵�ʱ�򣬲�����·��Ѳ��ķ���
            disp([num2str(t),'ʱ��',num2str(i),'�Ŵ�����С�ڵ�λʱ��·��']);
            row=row+1;
            step_temp=deltaPos;
            while step_temp<Boat(i).speed
                delta_pos0=Boat(i).path(row+1,:)-Boat(i).path(row,:);
                delta_pos=norm(delta_pos0);
                step_temp=step_temp+delta_pos;
                row=row+1;
            end
            Boat(i).pos=Boat(i).path(row,:);
        end
        Boat(i).HisPos=[Boat(i).HisPos;t,Boat(i).pos];
        deltaPos=Boat(i).pos-LastPos;
        Boat(i).COG_deg = NavAng(deltaPos);
        Boat(i).COG_rad = Boat(i).COG_deg/180*pi;
        Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
        Boat(i).Current_row=row;
        
    elseif Boat(i).reach==0  %�Ѿ������յ�
        Boat(i).pos = Boat(i).pos;
        Boat(i).HisPos=[Boat(i).HisPos;t,Boat(i).pos];
        Boat(i).COG_deg = Boat(i).COG_deg;
        Boat(i).COG_rad = Boat(i).COG_rad;
        Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
        
    elseif Boat(i).FM_lable==0  %û�о��߹���״̬
        
        Boat(i).pos = [Boat(i).pos(1)+Boat(i).speed*sind(Boat(i).COG_deg),Boat(i).pos(2)+Boat(i).speed*cosd(Boat(i).COG_deg)];
        Boat(i).HisPos=[Boat(i).HisPos;t,Boat(i).pos];
        Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
    end
    
    if norm(Boat(i).pos-Boat(i).goal)<=2*Res %������ǰ������ͬһ�����������Ϊ��������Ŀ���
        disp([num2str(t),'ʱ��',num2str(i),'�Ŵ�����Ŀ���']);
        Boat(i).reach=0;
    end
    
end
end

