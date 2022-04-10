function [outputArg1] = collision(x,y,theta,ku)
L=3.569;
CX0=x+1.551/2*sin(theta)-0.544*cos(theta);%�Һ�
DX0=x-1.551/2*sin(theta)-0.544*cos(theta);%���
AX0=DX0+L*cos(theta);%��ǰ
BX0=CX0+L*cos(theta);%��ǰ
CY0=y-1.551/2*cos(theta)-0.544*sin(theta);
DY0=y+1.551/2*cos(theta)-0.544*sin(theta);
AY0=DY0+L*sin(theta);
BY0=CY0+L*sin(theta);
outputArg1=0;
safe_r = 0.03;
safe_f = 0.03;
%% ��ⰲȫ
px=0;
py=0;
S_triangle=abs((AX0 - px)*(BY0 - py) - (AY0 - py)*(BX0 - px)) * 0.5 + abs((BX0 - px)*(CY0 - py) - (BY0 - py)*(CX0 - px)) * 0.5 + abs((CX0 - px)*(DY0 - py) - (CY0 - py)*(DX0 - px)) * 0.5 + abs((DX0 - px)*(AY0 - py) - (DY0 - py)*(AX0 - px)) * 0.5 ;
if S_triangle<=(3.569)*1.551
    outputArg1=20000;
end
%% ��ⰲȫ
if CX0<=-ku+safe_r || DX0<=-ku+safe_r
    outputArg1=20000;
end
if BY0<=-2 || CY0<=-2
    outputArg1=20000;
end
obs=[-safe_f,0,-safe_f,-2,5,-2,5,0];%ԭ�㿪ʼ��ʱ��
px=BX0;
py=BY0;
S_triangle=abs((obs(7)-px)*(obs(6)-py)-(obs(8)-py)*(obs(5)-px))*0.5 +abs((obs(5)-px)*(obs(4)-py)-(obs(6)-py)*(obs(3)-px))*0.5+abs((obs(3)-px)*(obs(2)-py)-(obs(4)-py)*(obs(1)-px))*0.5+abs((obs(1)-px)*(obs(8)-py)-(obs(2)-py)*(obs(7)-px))*0.5 ;
if S_triangle<=2*(5+safe_f)
    outputArg1=20000;
end
px=AX0;
py=AY0;
S_triangle=abs((obs(7)-px)*(obs(6)-py)-(obs(8)-py)*(obs(5)-px))*0.5 +abs((obs(5)-px)*(obs(4)-py)-(obs(6)-py)*(obs(3)-px))*0.5+abs((obs(3)-px)*(obs(2)-py)-(obs(4)-py)*(obs(1)-px))*0.5+abs((obs(1)-px)*(obs(8)-py)-(obs(2)-py)*(obs(7)-px))*0.5 ;
if S_triangle<=2*(5+safe_f)
    outputArg1=20000;
end

if x<-ku+safe_r+0.544
    outputArg1 = 20000;
end

end

