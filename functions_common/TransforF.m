function [a,b,angle_x,speed_X] = TransforF(angle,speed,angle_x0,speed_X0)
a=angle(end);
b=speed(end);

angle_x=angle_x0;
speed_X=speed_X0;
%{
dt=0.05;
A1=[-8.33200000000000,-36.6000000000000,-74.9200000000000,-248.200000000000;1,0,0,0;0,1,0,0;0,0,1,0];
B1=[1;0;0;0];
C1=[8.56700000000000,26.9000000000000,78.3400000000000,248.600000000000];
D1=0;

x0=angle_x0;
x=angle_x0;
u=angle;
x_=A1*x0+B1*u;
x=x+dt*x_;
angle_r=C1*x+D1*u;
%%
A2=[-4.02792341944679,-27.0925219068148,-46.4786667580669,-52.7962636344819;1,0,0,0;0,1,0,0;0,0,1,0];
B2=[1;0;0;0];
C2=[0,0,25.7485529211573,47.8541040461444];
D2=0;

X0=speed_X0;
X=speed_X0;
U=speed;

X_=A2*X0+B2*U;
X=X+dt*X_*1;
Y=C2*X+D2*U;

a=angle_r;
b=Y;
angle_x=x;
speed_X=X;
%}
end

