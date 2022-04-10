function [reward_total,reward_component]=RewardFunction(x,y,theta,Action,Risk)
reward1=0;
L=3.569;
CX0=x+1.551/2*sin(theta)-0.544*cos(theta);%右后
DX0=x-1.551/2*sin(theta)-0.544*cos(theta);%左后
AX0=DX0+L*cos(theta);%左前
BX0=CX0+L*cos(theta);%右前
CY0=y-1.551/2*cos(theta)-0.544*sin(theta);
DY0=y+1.551/2*cos(theta)-0.544*sin(theta);
AY0=DY0+L*sin(theta);
BY0=CY0+L*sin(theta);
%% 1.是否成功（失败情况：超时；高于起始位姿；低于目标位姿；碰撞(但时间短)）
reward0 = 0;
if (-2+1.551)<AY0 && AY0<(0) && (-2+1.551)<DY0 && DY0<(0) && abs(theta)<=3*pi/180
    reward0=10000;
end
% reward1 = -20000/(1+exp(-1.1*abs(y-(-0.85))))+20000; % 本来是1.1

reward1 = 20000 .* exp(-abs(y-(-0.9))/0.08);
% reward1 = 20000 .* exp(-abs(y-(-0.9))/0.08) .* (y>=-0.9) + 20000 .* exp(-abs(y-(-0.9))/0.01) .* (y<-0.9);


% 2.泊车时间：TODO：过高，过低，短时间碰撞，是不是也要比超时要好？
time=length(Action)*0.05;
reward2=1000-40*time;%超过25秒判定失败

% 3.最终航向角(0-10,000)
% reward3 = -20000./(1+exp(-0.3*abs(theta-0)*180/pi))+20000;
reward3 = 20000 .* exp(-abs(theta)/0.01);

% 4.是否碰撞
reward4 = -Risk;

% ----------
% 在reward4 中增加指令波动的惩罚
reward_action_all = cumsum(abs(Action(2:end)-Action(1:end-1))) * (-1);
% reward4 = reward4 

reward_total = reward0 + reward1+reward2+reward3+reward4 + reward_action_all(end);
reward_component=[reward0, reward1,reward2,reward3,reward4,reward_total];
end
%{
%% 1.超时停止
    %% 2.高于起始位姿停止
    %% 3.低于目标位姿停止
    %% 4.达到目标停止
    %% 5.碰撞停止（TODO：是否有更好办法？）
%}