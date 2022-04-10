% 先画环境
function plot_simulation_result(lenKu, ex, jiange)
% figure
% Input: lenKu 库位长度 
% 画库位
if nargin < 3
    jiange = 10;
end

% 下面是对轨迹进行画图
trajectory_x = ex(1,:);
trajectory_y = ex(2,:);
navigate_angle = ex(3,:);
length_traj=length(trajectory_x);
hold on

% figure(1)
plot(trajectory_x,trajectory_y,'color','k');
% plot(trajectory_x,trajectory_y,'color','r');
% title(tt)
axis equal
% axis([-0.5 9 -1 6])
% xlabel('x/ k')
% ylabel('y/ k')
grid on
set(gca,'FontSize',14,'Fontname', 'Times New Roman');

plot_box = 1;
if(plot_box)
    L = 3.569; % chechang
    delta = jiange;
    for ii5=2:delta:length_traj
        %% 计算四个x  左下，左上，右上，右下
        x1(ii5)=trajectory_x(ii5)+1.551/2*sin(navigate_angle(ii5))-0.544*cos(navigate_angle(ii5));
        x2(ii5)=trajectory_x(ii5)-1.551/2*sin(navigate_angle(ii5))-0.544*cos(navigate_angle(ii5));
        x3(ii5)=x2(ii5)+L*cos(navigate_angle(ii5));
        x4(ii5)=x1(ii5)+L*cos(navigate_angle(ii5));
        y1(ii5)=trajectory_y(ii5)-1.551/2*cos(navigate_angle(ii5))-0.544*sin(navigate_angle(ii5));
        y2(ii5)=trajectory_y(ii5)+1.551/2*cos(navigate_angle(ii5))-0.544*sin(navigate_angle(ii5));
        y3(ii5)=y2(ii5)+L*sin(navigate_angle(ii5));
        y4(ii5)=y1(ii5)+L*sin(navigate_angle(ii5));
        % 画矩形
        line([x1(ii5),x2(ii5)],[y1(ii5),y2(ii5)],'LineWidth',1,'color','r')
        line([x2(ii5),x3(ii5)],[y2(ii5),y3(ii5)],'LineWidth',1,'color','r')
        line([x3(ii5),x4(ii5)],[y3(ii5),y4(ii5)],'LineWidth',1,'color','r')
        line([x4(ii5),x1(ii5)],[y4(ii5),y1(ii5)],'LineWidth',1,'color','r')
    end
    plot_env(lenKu);
    % 画最后一个
    for ii5=length_traj:length_traj
        %% 计算四个x  左下，左上，右上，右下
        x1(ii5)=trajectory_x(ii5)+1.551/2*sin(navigate_angle(ii5))-0.544*cos(navigate_angle(ii5));
        x2(ii5)=trajectory_x(ii5)-1.551/2*sin(navigate_angle(ii5))-0.544*cos(navigate_angle(ii5));
        x3(ii5)=x2(ii5)+L*cos(navigate_angle(ii5));
        x4(ii5)=x1(ii5)+L*cos(navigate_angle(ii5));
        y1(ii5)=trajectory_y(ii5)-1.551/2*cos(navigate_angle(ii5))-0.544*sin(navigate_angle(ii5));
        y2(ii5)=trajectory_y(ii5)+1.551/2*cos(navigate_angle(ii5))-0.544*sin(navigate_angle(ii5));
        y3(ii5)=y2(ii5)+L*sin(navigate_angle(ii5));
        y4(ii5)=y1(ii5)+L*sin(navigate_angle(ii5));
        % 画矩形
        line([x1(ii5),x2(ii5)],[y1(ii5),y2(ii5)],'LineWidth',2,'color','b')
        line([x2(ii5),x3(ii5)],[y2(ii5),y3(ii5)],'LineWidth',2,'color','b')
        line([x3(ii5),x4(ii5)],[y3(ii5),y4(ii5)],'LineWidth',2,'color','b')
        line([x4(ii5),x1(ii5)],[y4(ii5),y1(ii5)],'LineWidth',2,'color','b')
    end

end

grid off
box on

end