function [x_new,y_new,theta_new,ds_new]=kinematic(x,y,theta,v_real_NN,Angle, ds)%
chuan=15.88;

% E50参数
% global qulv_angle;
% qulv = interp1(qulv_angle(:,1), qulv_angle(:,2), Angle);

% Carsim参数
angleReal = Angle;
% carsim的标定传动比
if angleReal > 480
    chuan = 15.8549;
elseif angleReal > 380
    chuan = 15.8861;
elseif angleReal > 280
    chuan = 15.9079;
elseif angleReal > 180
    chuan = 15.8766;  
elseif angleReal > -180
    chuan = 15.8780; 
elseif angleReal > -280
    chuan = 15.8724; 
elseif angleReal > -380
    chuan = 15.9082;  
elseif angleReal > -480
    chuan = 15.8848; 
else
    chuan = 15.8555;
end


% chuan=16.68;% E50参数
if Angle>530
    Angle=530;
end
if Angle<-530
    Angle=-530;
end
if v_real_NN>1
    v_real_NN=1;
end
if v_real_NN<-1
    v_real_NN=-1;
end
fai=Angle/chuan*pi/180;
L=2.305;
dt=0.05;
%%
v_real=v_real_NN;
x_dot=v_real.*cos(theta);
y_dot=v_real.*sin(theta);
theta_dot=v_real.*tan(fai)/L;
% theta_dot = v_real*qulv;

x_new=x+x_dot*dt;
y_new=y+y_dot*dt;
theta_new=theta+theta_dot*dt;

ds_new = ds + abs(v_real) * dt;

end