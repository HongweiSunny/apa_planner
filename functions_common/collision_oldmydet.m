% 此函数用于检测带倒角的八边形车辆轮廓是否与障碍车碰撞

function r = collision(x,y,th,ku)

r = 0; % 碰撞风险

persistent a b c d e f g p1 p2 p3 p4 p5 p6 p7 p8 p9 p_all ego_A obs obs_A
persistent Tvr_inv 
% ---------------- 前后安全距离
persistent safe_r safe_f
if isempty(a)
    
% 先计算出8个角点的坐标
a = 0.16;
b = 0.365;
c = 1.551 - 2 * b;
d = 3.03;
e = 0.38;
f = 1.551 - 2 * e;
g = 0.544;

% 齐次坐标
p1 = [0 b 1];
p2 = [0 b+c 1];
p3 = [a 2*b+c 1];
p4 = [a+d 2*b+c 1];
p5 = [a+e+d f+e 1];
p6 = [a+d+e e 1];
p7 = [a+d 0 1];
p8 = [a 0 1 ];
p9 = [g b+c/2 atan((b+c/2)/g)]; % 后轴中心
p_all = [p1; p2; p3;p4;p5;p6;p7;p8];

% 车的面积
ego_A = (2*b+c)*(a+e+d) - a*b - e*e;
% 障碍物坐标
obs = [0 0; 5 0; 5 -2; 0 -2]';
% 障碍物面积
obs_A = 10;

Tvr_inv  = inv([1 0 g; 0 1 b+c/2; 0 0 1]);
safe_r = 0.01;
safe_f = 0.01;

end

% 旋转车辆
Twr = [cos(th) -sin(th) x; sin(th) cos(th) y; 0 0 1];

Twv = Twr*Tvr_inv;
p_all_w = Twv * p_all';




if ~isempty(  find(p_all_w(1,:)<-ku+safe_r) ) % 后面的碰撞直接判断x轴坐标
    r = 20000;
else
    r = 0;
end
% -------------
if r == 0 % 后面没有撞
    % 车于障碍物
    for k = [6 8] % 本应循环8次  但是往右泊车只有右边的点会撞到 为了减小计算量 只计算6 7 8 1 
        S = 0.5 *  mydet( p_all_w(1, k),     p_all_w(2, k), ...
                                obs(1, 1)   ,       obs(2, 1),...
                                 obs(1, 2), obs(2, 2) )  + ...
                0.5 *  mydet( p_all_w(1, k)  ,   p_all_w(2, k), ...
                                obs(1, 2) ,         obs(2, 2),...
                                 obs(1, 3), obs(2, 3))  +...
                0.5 *  mydet( p_all_w(1, k)   ,  p_all_w(2, k), ...
                                obs(1, 3)     ,     obs(2, 3) ,...
                                 obs(1, 4) ,obs(2, 4) )  +...
                0.5 *  mydet( p_all_w(1, k) ,    p_all_w(2, k), ...
                                obs(1, 4)   ,       obs(2, 4),...
                                 obs(1, 1) ,obs(2, 1) ) ;
%         % 计算角度
%         an = 0;
        if S <= obs_A + 0.01 % 安全值设置为0.01 比较小 大于  说明车的某个点在障碍车矩形内部
            r = 20000;
            break; % 退出循环
        else
            continue;
        end
    end
end

% 障碍物于车 本应循环4次 但是只有0 0这个点才可能撞到
if r == 0
    nn = 1;
%     S2 = 0;
%     for zz = 1:8 % 8个自车角点
%         S2 = S2 + 0.5 * abs( mydet([obs(1, nn) obs(2, nn) 1; ...
%                         p_all_w(1,zz) p_all_w(2,zz) 1;...
%                         p_all_w(1,mod(zz,8)+1), p_all_w(2,mod(zz,8)+1) 1]) );
%     end
    S2 = mydet(obs(1, nn), obs(2, nn), ...
                        p_all_w(1,1), p_all_w(2,1) ,...
                        p_all_w(1,2), p_all_w(2,2) )  + ...
        mydet(obs(1, nn), obs(2, nn), ...
                        p_all_w(1,2), p_all_w(2,2), ...
                        p_all_w(1,3), p_all_w(2,3) ) +...
        mydet(obs(1, nn), obs(2, nn),...
                        p_all_w(1,3), p_all_w(2,3),...
                        p_all_w(1,4), p_all_w(2,4)) +...               
         mydet(obs(1, nn), obs(2, nn) , ...
                        p_all_w(1,4) ,p_all_w(2,4) ,...
                        p_all_w(1,5), p_all_w(2,5) ) +...              
        mydet(obs(1, nn) ,obs(2, nn),  ...
                        p_all_w(1,5), p_all_w(2,5) ,...
                        p_all_w(1,6), p_all_w(2,6) ) +... 
         mydet(obs(1, nn),obs(2, nn),  ...
                        p_all_w(1,6) ,p_all_w(2,6), ...
                        p_all_w(1,7), p_all_w(2,7) ) +...               
          mydet(obs(1, nn),obs(2, nn),...
                        p_all_w(1,7),p_all_w(2,7),...
                        p_all_w(1,8), p_all_w(2,8)) +...  
          mydet(obs(1, nn),obs(2, nn), p_all_w(1,8),p_all_w(2,8),...
                        p_all_w(1,1), p_all_w(2,1));  
                        
    if 0.5*S2 <= ego_A + 0.01
        r = 20000;
    end
end

    
  
end % func end

function d = mydet(x1,y1,x2,y2,x3,y3)
d = abs(x2*y3-y2*x3 + x3*y1-y3*x1 + x1*y2-y1*x2);
end