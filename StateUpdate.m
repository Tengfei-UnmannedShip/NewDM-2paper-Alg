function Boat = StateUpdate(Boat0,Boat_Num,t,Res)
% 每个时刻的场景状态更新，分成不同的情况
Boat=Boat0;

for i=1:1:Boat_Num
    if Boat(i).FM_lable~=0 && Boat(i).reach==1 %即已经开始决策但没有到终点
        %由于FMM方法的特点，有些点之间步长特别长，因此需要分成两种情况
        row=Boat(i).Current_row;
        deltaPos=Boat(i).path(row+1,:)-Boat(i).pos;
        LastPos=Boat(i).pos;
        if  norm(deltaPos)>Boat(i).speed
            %步长大于单位时间船舶走的路程的时候，需要沿着规划之后的两点之间的线段找点
            disp([num2str(t),'时刻',num2str(i),'号船步长大于单位时间路程']);
            x_temp=Boat(i).pos(1);
            y_temp=Boat(i).pos(2);
            pos_temp0=[x_temp,y_temp];
            step_temp=0;
            while step_temp<Boat(i).speed
                x_l=Boat(i).path(row:row+1,1);
                y_l=Boat(i).path(row:row+1,2);
                %采取插值函数找点
                if x_l(1)==x_l(2)  %一条竖直线
                    x_temp=x_temp;
                    y_temp=y_temp+1;   %每次走1米
                elseif y_l(1)==y_l(2)  %一条水平线
                    x_temp=x_temp+1;     %每次走1米
                    y_temp=y_temp;
                else   %一条常规的斜线
                    x_temp=x_temp+1;    %每次x走1米
                    y_temp=interp1(x_l,y_l,x_temp,'spline');
                end
                pos_temp=[x_temp,y_temp];
                delta_pos0=pos_temp-pos_temp0;
                delta_pos=norm(delta_pos0);
                step_temp=step_temp+delta_pos;
                pos_temp0=pos_temp;
            end
            Boat(i).pos=pos_temp;
        else    %步长小于单位时间路程的时候，采用沿路径巡点的方法
            disp([num2str(t),'时刻',num2str(i),'号船步长小于单位时间路程']);
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
        
    elseif Boat(i).reach==0  %已经到达终点
        Boat(i).pos = Boat(i).pos;
        Boat(i).HisPos=[Boat(i).HisPos;t,Boat(i).pos];
        Boat(i).COG_deg = Boat(i).COG_deg;
        Boat(i).COG_rad = Boat(i).COG_rad;
        Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
        
    elseif Boat(i).FM_lable==0  %没有决策过的状态
        
        Boat(i).pos = [Boat(i).pos(1)+Boat(i).speed*sind(Boat(i).COG_deg),Boat(i).pos(2)+Boat(i).speed*cosd(Boat(i).COG_deg)];
        Boat(i).HisPos=[Boat(i).HisPos;t,Boat(i).pos];
        Boat(i).HisCOG=[Boat(i).HisCOG;Boat(i).COG_rad,Boat(i).COG_deg];
    end
    
    if norm(Boat(i).pos-Boat(i).goal)<=2*Res %本船当前距离在同一个格子里，即认为本船到达目标点
        disp([num2str(t),'时刻',num2str(i),'号船到达目标点']);
        Boat(i).reach=0;
    end
    
end
end

