function [] = MCTS_init(x,y,t,ku)
for ii=1:1:1%循环起始位姿
    for jj=1:1:1%循环起始位姿
        for i=1:1:1
            [ex,PI,Reward0]=executeEpisode([x,y,t],ku);%[1.3,1.3,0],Ku
            example{1,i}=ex;
            X=ex(1,:);
            Y=ex(2,:);
            xx000(i)=X(end);
            yy000(i)=Y(end);
            theta(i)=ex(3,end)*180/pi;%单位是度
            Theta=ex(3,:);
        end
        Var_x(ii,jj)=var(xx000);
        Var_y(ii,jj)=var(yy000);
        Var_t_deg(ii,jj)=var(theta);
        Mean_t_deg(ii,jj)=mean(theta);
        Mean_y(ii,jj)=mean(yy000);
    end
end
% save ControlResult_Kine.mat
testExamples1=ex;%hou8qian1trainDate
trajectory_x=testExamples1(1,:)+5;%5米库位
trajectory_y=testExamples1(2,:);
navigate_angle=testExamples1(3,:);
length_traj=length(trajectory_x);
L=3.569;
for ii5=1:1:length_traj
    x1(ii5)=trajectory_x(ii5)+1.551/2*sin(navigate_angle(ii5))-0.544*cos(navigate_angle(ii5));%XC
    x2(ii5)=trajectory_x(ii5)-1.551/2*sin(navigate_angle(ii5))-0.544*cos(navigate_angle(ii5));%XD
    x3(ii5)=x2(ii5)+L*cos(navigate_angle(ii5));%XA
    x4(ii5)=x1(ii5)+L*cos(navigate_angle(ii5));%XB
    y1(ii5)=trajectory_y(ii5)-1.551/2*cos(navigate_angle(ii5))-0.544*sin(navigate_angle(ii5));%YC
    y2(ii5)=trajectory_y(ii5)+1.551/2*cos(navigate_angle(ii5))-0.544*sin(navigate_angle(ii5));%YD
    y3(ii5)=y2(ii5)+L*sin(navigate_angle(ii5));%YA
    y4(ii5)=y1(ii5)+L*sin(navigate_angle(ii5));%YB
end
NENE=floor(length_traj/4);
x=ex(1,1:NENE*4);
y=ex(2,1:NENE*4);
theta=ex(3,1:NENE*4);
AX=x3(1:NENE*4);
BX=x4(1:NENE*4);
CX=x1(1:NENE*4);
DX=x2(1:NENE*4);
AY=y3(1:NENE*4);
BY=y4(1:NENE*4);
CY=y1(1:NENE*4);
DY=y2(1:NENE*4);
ttff=(NENE*4-1)*0.05;
phy=-ex(5,1:NENE*4)/15.8795*pi/180;
a(1)=0;
for ii=1:1:NENE*4-1
    a(ii+1)=(ex(6,ii+1)-ex(6,ii))/0.05;
end
v=ex(6,1:NENE*4);
w(1)=0;
for ii=1:1:NENE*4-1
    w(ii+1)=-(ex(5,ii+1)-ex(5,ii))/0.05/15.8795*pi/180;
end
save data.mat ex
save NN_output.mat x y theta AX BX CX DX AY BY CY DY ttff phy a v  w NENE
end

