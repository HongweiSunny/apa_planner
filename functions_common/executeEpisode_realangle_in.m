function [examples,PI,Reward,hist_defaultpolicy_reward,hist_max_reward,ifcollision,hist_success,hist_r, poseStop, updateCount, success_final_flag]= executeEpisode(s,ku, xs, ys,real_angle_in)%初始位置/库位

%%
global qulv_angle
load('.\Data_param\qulv_angle_curve_new_E50.mat');
qulv_angle = qulv_angle_E50;

updateCount = 0;
poseStop = [0;0;0]';
y_ini=s(2);
SS=[];
Action_angle=[0];
Action_v=[0];
Action_real=[real_angle_in];
state_angle=[0;0;0;0];
state_velocity=[0;0;0;0;0];
vorder=-0.0;
%%
angle_xx0=zeros(2,1);%根节点
speed_XX0=zeros(2,1);%根节点
RISK=0;%已执行动作风险。这一项的作用只是传给DEFAULTPOLICY
angle_xx0_root=angle_xx0;speed_XX0_root=speed_XX0;
%%
aaction_if_success=0;
once_success=0;
hist_success=[];
hist_r=[];
rnowmax=-1e10;
rnowmax_y = -1e10;
rmaxlabel=-1e10;
search_stop=0;
ITER=1;
hist_defaultpolicy_reward=[];
hist_max_reward=[];
% if_success=0;
r_detail_all = [];
stop_reason_all = [];
success_once = 0; 
success_poseSeq = [];
success_poseChange = [];
while search_stop<1%TODO：指令执行完毕
    if ITER==1
        s0=s;
    else
        s0=S0;
    end
    S=[s0';vorder];
    SS=[SS S];
    if RISK==0
        RISK=collision(s0(1),s0(2),s0(3),ku);
    end
    
    
    % 先往下搜索一层
    r = -1e10;
    r_detail = zeros(1,6);
    r_ind = 1;
    p_s_a_15 = VA_NN_201126([s0(1);s0(2);s0(3);Action_real(end);vorder]);
%     [~, sorted_ind] = sort(p_s_a_15, 'ascend');
    [~, sorted_ind] = sort(p_s_a_15, 'descend');
    for iii = 1:15
        ind = sorted_ind(iii);
        x=s0(1);
        y=s0(2);
        theta=s0(3);
        
%         % 速度
%         p_s_a2=speedNN_1110([x;y;theta; vorder; Action_real(end)]); % 原来的这个输入特征
%         select=find(p_s_a2>=max(p_s_a2));
%         cixu=select(1);
%         action_v=(cixu-2)*0.3;
%         vorder_next=vorder+action_v*0.05;
%         if vorder_next<-1
%             vorder_next = -1;
%         end
%         if vorder_next>0.5
%             vorder_next=0.5;
%         end
%         % 转角
%         action_next = Action_real(end) + 10*ind-25;
%         action_next = min(530, action_next);
%         action_next = max(-530, action_next);
        % 速度
%         p_s_a_15 = VA_NN_201126(x,y,theta,vorder,Action_real(end));
        
        v_ind = ceil(ind/5);
        a_ind = ind - v_ind * 5 + 5;
        % 车速
        action_v=(v_ind-2)*0.3;
        vorder_next = vorder + action_v*0.05;
        if vorder_next<-1
            vorder_next = -1;
        end
        if vorder_next>0.5
            vorder_next=0.5;
        end
        % 转角
        action_next = Action_real(end) + 10*a_ind-30;
        action_next = min(530, action_next);
        action_next = max(-530, action_next);
        
        [x_next, y_next, theta_next] = kinematic(x,y,theta,vorder_next,action_next);
        
        % 给默认策略一个初始的状态
        s0_next = [x_next y_next theta_next];
        [rxx,r_detailxx, stop_reasonxx, poseChangexx, aanglexx,vorders0xx,if_successxx, poseSeqDepoxx, CollisionDepoxx]=DEFAULTPOLICY2(s0_next,y_ini,[Action_real action_next],ITER+1,RISK,ku,angle_xx0,speed_XX0,vorder_next, xs, ys);
        
        % ------- 改变衡量条件
%         if ( if_successxx &&  r_detail(2) < r_detailxx(2) ) || r<rxx
        if r<rxx
            r = rxx;
            r_detail = r_detailxx;
            stop_reason = stop_reasonxx;
            poseChange = poseChangexx;
            aangle = [action_next aanglexx];
            vorders0 = [vorder_next vorders0xx];
            if_success = if_successxx;
            poseSeqDepo = poseSeqDepoxx;
            CollisionDepo = CollisionDepoxx;
            r_ind = ind;
        end
        % figure(11111)
        % hold on
        % %plot(poseSeqDepoxx(1,:), poseSeqDepoxx(2,:))
    end
    
    
    % -----
    r_detail_all = [r_detail_all; r_detail];
    stop_reason_all = [stop_reason_all stop_reason];
    
    % 查看指令的形状
%     figure(2001)
%     cla
%     title('angle_order')
%     %plot(aangle)
%     figure(2002)
%     cla
%     title('velo_order')
%     %plot(vorders0)


    hist_success=[hist_success,if_success];
    hist_r=[hist_r,r];
    %当前动作链不成功时，每过10步强制刷新
%     if aaction_if_success==0 && mod(ITER,10)==0
%         rnowmax=-1e10;
%     end
%     if aaction_if_success==1
%         once_success=1;
%     end
%     if r>rnowmax || if_success || mod(ITER,10)==0 % 验证目前的网络是否能稳定地产生可行的轨迹
%     if ( if_success && rnowmax_y < r_detail(2)) || r>rnowmax  
    if r>rnowmax
        rnowmax=r;
        rnowmax_y = r_detail(2);
        aaction=aangle;
        vorders=vorders0;
%         aaction_if_success=if_success;
%     end
%     if rnowmax<r 
%         rmaxlabel=rnowmax;
        action=aaction(1);
        vorder=vorders(1);
        poseSeqUsed = poseSeqDepo;
        cx=1;
%         disp('updated')
    else
        if cx+1<=length(aaction)%如果还有未执行，继续执行链。否者保持上一个
            cx=cx+1;
            action=aaction(cx);
            vorder=vorders(cx);
        end
    end
   
    
    hist_defaultpolicy_reward=[hist_defaultpolicy_reward r];
    hist_max_reward=[hist_max_reward rnowmax];
    Action_angle=[Action_angle,action];
    Action_v=[Action_v,vorder];
    x=s0(1);
    y=s0(2);
    theta=s0(3);
    ITER=ITER+1;
    [x_new,y_new,theta_new]=kinematic(x,y,theta,vorder,action);%
    Action_real=[Action_real,action];
    S0=[x_new,y_new,theta_new];
    [search_stop]=stop_judge_special_taskover(x_new,y_new,theta_new,y_ini,ku,ITER,cx,aaction); 
    %TODO：指令执行完毕是不是还不能停车
    angle_xx0_root=[angle_xx0_root angle_xx0];
    speed_XX0_root=[speed_XX0_root speed_XX0];
    
    figure(2000)
    cla
    xlim([-6,5])
    ylim([-2,4])
    plot(poseSeqUsed(1,:), poseSeqUsed(2,:),'k--', 'LineWidth',2)
    hold on 
    plot(x_new, y_new, 'ro', 'LineWidth', 3)
    plot(poseSeqDepo(1,:), poseSeqDepo(2,:) )
    plot(poseSeqDepo(1,end), poseSeqDepo(2,end), 'go')
    hold on
    
%     r_detail_all;
%     figure
%     stem(r_detail_all(:,end))
%     stop_reason_all;
%     figure
%     stem(stop_reason_all(:,end))
end
[~,reward_overall]=RewardFunction(x_new,y_new,theta_new,Action_angle,RISK);
ifcollision=0;
% if reward_overall(4)~=0
if  collision(x_new, y_new, theta_new, ku)
    ifcollision = 1;
end
% end


LL=length(SS(1,:)); 
examples=[SS(1:3,1:LL); Action_angle(1:LL);Action_v(1:LL);...
    Action_angle(2:LL+1) - Action_angle(1:LL);...
      Action_v(2:LL+1) - Action_v(1:LL)];
PI = [Action_angle(2:LL+1) - Action_angle(1:LL);...
      Action_v(2:LL+1) - Action_v(1:LL);];

% PI = [Action_angle(1:(length(Action_angle)-1));Action_real(1:(length(Action_angle)-1))];
Reward=reward_overall;

poseStop = [x_new,y_new,theta_new]';
success_final_flag = reachTarget(x_new, y_new, theta_new, ku);
end