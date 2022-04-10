function plot_E50_env(ku, ex, delta)

if nargin == 2
    delta = 10;
end
    

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
p9 = [g b+c/2 1];
p_all = [p1; p2; p3;p4;p5;p6;p7;p8;p1];
% figure
% plot(p_all(:,1), p_all(:,2),'k-','lineWidth', 2)
% hold on
% plot(p9(1),p9(2),'ro','LineWidth',3);

% ---- 旋转车辆
% figure(1)
hold on
plot_env(ku)
% x = -3.568; y = -0.8; th = pi/8;
for ii = 1:delta:size(ex,2)-1
    x = ex(1,ii); y = ex(2,ii); th = ex(3,ii);
    Twr = [cos(th) -sin(th) x; sin(th) cos(th) y; 0 0 1];
    Tvr = [1 0 g; 0 1 b+c/2; 0 0 1];
    Twv = Twr*inv(Tvr);
    p_all_w = Twv * [[p_all; p9]]';

%     figure(1)
    plot(p_all_w(1,1:end-1), p_all_w(2,1:end-1),'g-','lineWidth', 1)
    hold on
    plot(p_all_w(1,end), p_all_w(2,end),'kp')
    axis equal
    
end

for ii = size(ex,2)
    x = ex(1,ii); y = ex(2,ii); th = ex(3,ii);
    Twr = [cos(th) -sin(th) x; sin(th) cos(th) y; 0 0 1];
    Tvr = [1 0 g; 0 1 b+c/2; 0 0 1];
    Twv = Twr*inv(Tvr);
    p_all_w = Twv * [[p_all; p9]]';

%     figure(1)
    plot(p_all_w(1,1:end-1), p_all_w(2,1:end-1),'k-','lineWidth', 1.5)
    hold on
    plot(p_all_w(1,end), p_all_w(2,end),'ko')
    axis equal
    box on
end

for ii = 1
    x = ex(1,ii); y = ex(2,ii); th = ex(3,ii);
    Twr = [cos(th) -sin(th) x; sin(th) cos(th) y; 0 0 1];
    Tvr = [1 0 g; 0 1 b+c/2; 0 0 1];
    Twv = Twr*inv(Tvr);
    p_all_w = Twv * [[p_all; p9]]';

%     figure(1)
    plot(p_all_w(1,1:end-1), p_all_w(2,1:end-1),'k-','lineWidth', 1.5)
    hold on
    plot(p_all_w(1,end), p_all_w(2,end),'kp')
    axis equal
    box on
end

hold on
plot(ex(1,:), ex(2,:))
% 显示小格子
    set(gca, 'XMinorGrid','on');
    set(gca, 'YMinorGrid','on');
end