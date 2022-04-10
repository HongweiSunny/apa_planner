function [outputArg1] = collision_triangle(AX0,AY0,BX0,BY0,CX0,CY0,DX0,DY0,px,py)
%������ǰ/��ǰ�ǵ㣺[AX0,AY0],     [BX0,BY0]
%�����Һ�/���ǵ㣺[CX0,CY0],     [DX0,DY0]
%���жϵ�PPP:[px,py]
S_triangle=abs((AX0 - px)*(BY0 - py) - (AY0 - py)*(BX0 - px)) * 0.5 + abs((BX0 - px)*(CY0 - py) - (BY0 - py)*(CX0 - px)) * 0.5 + abs((CX0 - px)*(DY0 - py) - (CY0 - py)*(DX0 - px)) * 0.5 + abs((DX0 - px)*(AY0 - py) - (DY0 - py)*(AX0 - px)) * 0.5 ;
outputArg1=0;
if S_triangle<=(3.569)*1.544
    outputArg1=10000;
end
end

