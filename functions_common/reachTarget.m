function [IfreachTarget] = reachTarget(x,y,theta,lenku)
%航向角小于1度，且自车轮廓在库位内部就认为达到目标区域
L=3.569;
CX0=x+1.551/2*sin(theta)-0.544*cos(theta);%右后
DX0=x-1.551/2*sin(theta)-0.544*cos(theta);%左后
AX0=DX0+L*cos(theta);%左前
BX0=CX0+L*cos(theta);%右前
CY0=y-1.551/2*cos(theta)-0.544*sin(theta);
DY0=y+1.551/2*cos(theta)-0.544*sin(theta);
AY0=DY0+L*sin(theta);
BY0=CY0+L*sin(theta);
IfreachTarget=0;
if theta<=0.1*pi/180
    if AY0<0 && DY0 <0
        if BY0>-2 && CY0>-2
            if AX0<0 && BX0<0
                if CX0>-lenku && DX0>-lenku
                    IfreachTarget=1;
                end
            end
        end
    end
end
end

