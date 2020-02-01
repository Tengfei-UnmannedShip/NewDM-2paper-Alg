function SCR = ShipDomain( Boat_x,Boat_y,Boat_theta,Boat_Speed,Shiplength,MapSize,Res,k )
%% 来自Ning Wang的基本SCR的船舶领域模型，注意原文中XY轴是颠倒的
[X,Y]=meshgrid(-MapSize(1)*1852:Res:MapSize(1)*1852,-MapSize(2)*1852:Res:MapSize(2)*1852);
r0 = 0.5;
k_AD = 10^(0.3591*log10(Boat_Speed)+0.0952);
k_DT = 10^(0.5441*log10(Boat_Speed)-0.0795);

Rfore = (1+1.34*sqrt(k_AD^2+0.25*k_DT^2))*Shiplength;
Raft = (1+0.67*sqrt(k_AD^2+0.25*k_DT^2))*Shiplength;
Rstarb = (0.2+k_DT)*Shiplength;
Rport = (0.2+0.75*k_DT)*Shiplength;
R = [Rfore Raft Rstarb Rport];
Sigma = R/((log(1/r0))^(1/k));

%把计算势场点坐标（X,Y）变换到船舶坐标系下点（BoatX,BoatY）
BoatX = (X-Boat_x)*cos(Boat_theta)+(Y-Boat_y)*sin(Boat_theta);
BoatY = (Y-Boat_y)*cos(Boat_theta)-(X-Boat_x)*sin(Boat_theta);

%注意原文中XY轴颠倒，本质是XY轴代表的四个方向的R = [Rfore Raft Rstarb Rport]的顺序
SCR= 100*exp(-(2*BoatX./((1+sign(BoatX))*Sigma(3)+(1-sign(BoatX))*Sigma(4))).^k-(2*BoatY./((1+sign(BoatY))*Sigma(1)+(1-sign(BoatY))*Sigma(2))).^k);

end

