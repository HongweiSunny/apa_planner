function [reward_stop,reward_detail,search_stop, poseChange, aangle,vorders,if_success, SS, CollisionDepo]=DEFAULTPOLICY2(state,y_ini,Action,ITER,Risk,ku,angle_x0,speed_X0,v_NN, xs, ys)

CollisionDepo = 0;
x=state(1);
y=state(2);
theta=state(3);
Angle=Action(end);
aangle=[];
vorders=[];
Iter=1;
search_stop=0;
SS=[];
v_NN0=v_NN;

% ������¼
poseChange = [];

% % ����������һ��  
% for index = 1:9
    
while (search_stop<1)
    %% ת��9����
%     p_s_a=steerNN_1110([x;y;theta;Angle;v_NN0]);% p_s_a=SteerNN([x;y;theta;Angle;(ITER+Iter-1)*0.05]);
%     p_s_a=SteerNN([x;y;theta;Angle]);
%     %% ���ո���ѡ���ɹ�55/81��
%     %     ycum=cumsum(p_s_a);
%     %     select=find(ycum>=rand);
%     %% ���ո��ʽ��һ���¶�ϵ����ϵ��1.2-56/81��ϵ��1.1-57/81��
%     tao=1.1;
%     p_s_a=p_s_a.^tao/(sum(p_s_a.^tao));
%     ycum=cumsum(p_s_a);
%     select=find(ycum>=rand);
%     %% �������ֵѡ��9/81��
%     %    select=find(p_s_a==max(p_s_a));
%     %%
%     cixu=select(1);
%     action=(cixu-5)*5;
%     Angle_old = Angle; % ��һ������
%     Angle=Angle+action;
%     if Angle>530
%         Angle=530;
%     end
%     if Angle<-530
%         Angle=-530;
%     end
%     aangle=[aangle Angle];
    
%     % ---------------
%     %% ���ٶ�3����
% %     p_s_a2=SpeedNN([x;y;theta;v_NN0]); % ԭ���������������
% 
%     % ���뷽����������Ϊ�������������
%     p_s_a2=speedNN_1110([x;y;theta; Angle_old;v_NN0]); % ԭ���������������
%     
%     %% ѡ�����ֵ
%     select=find(p_s_a2>=max(p_s_a2));
%     %% ���ѡ��
%     %ycum=cumsum(p_s_a2);
%     %select=find(ycum>=rand);
%     %%
%     cixu=select(1);
%     action_v=(cixu-2)*0.3;
%     v_NN=v_NN0+action_v*0.05;% v_NN=SpeedNN([x;y;theta;v_NN0;(ITER+Iter-1)*0.05]);
%     if v_NN>1
%         v_NN=1;
%     end
%     if v_NN<-1
%         v_NN=-1;
%     end
    
    
    % -------------15����
    p_s_a=VA_NN_201126([x;y;theta;Angle; v_NN0]);
    a = p_s_a;
    
    % ���ո���ѡ
    ycum=cumsum(p_s_a);
    select=find(ycum>=rand);
    % ���ո��ʽ��һ���¶�ϵ����ϵ��1.2-56/81��ϵ��1.1-57/81��
    tao=1.1;
    p_s_a1=p_s_a.^tao/(sum(p_s_a.^tao)); % Ŀǰû�н���¶�ϵ��
%     b = p_s_a1;
    
    %  boltzmann
%     p_s_a = exp(p_s_a)./(sum(exp(p_s_a)));
%     c = p_s_a;

    ycum=cumsum(p_s_a);
    select=find(ycum>=rand);
    ind = select(1);
    v_ind = ceil(ind/5);
    a_ind = ind - v_ind * 5 + 5;
    % ����
    action_v=(v_ind-2)*0.3;
    vorder_next = v_NN0 + action_v*0.05;
    if vorder_next<-1
        vorder_next = -1;
    end
    if vorder_next>0.5
        vorder_next=0.5;
    end
    v_NN = vorder_next;
    % ת��
    action_next = Angle + 10*a_ind-30;
    action_next = min(530, action_next);
    action_next = max(-530, action_next);
    Angle = action_next;
    aangle=[aangle Angle];
    
    
    
    
    % �ѻ�����¼����
    if sign(v_NN) ~= sign(v_NN0)
        poseChange = [poseChange [x;y;theta]];
    end

    
    [x_new,y_new,theta_new,~]=kinematic_ds(x,y,theta,v_NN,Angle,0);%
    v_NN0=v_NN;
    
    if Risk==0
        Risk=collision(x,y,theta,ku);
    end
    Iter=Iter+1;
    
    
    [search_stop]=stop_judge_special(x_new,y_new,theta_new,y_ini,ku,ITER+Iter);
    
    %% �����ײ�����ٷ���任������ɲ��ģ�飨������Ҫ�ж���ײ���ͣ��޸���ײ��֮ǰN��ָ�%% ���������ײ���ǳ�����0�������˳����棿
    Risk0 = collision(x_new,y_new,theta_new,ku);
    if collision(x_new,y_new,theta_new,ku)>0 
        CollisionDepo = Risk0;
        % ����ط�ͦ��ֵ� ����Ƿ�������ײ�϶�����������search_stop��1 ��ֱ�ӻ��˳� ����������ٶȾ�����û���õģ�
        %  v_NN=-v_NN��
        if v_NN>0
            v_NN=-0.1;
        else
            v_NN=0.1;
        end
        [x_new,y_new,theta_new,~]=kinematic_ds(x,y,theta,v_NN,Angle,0); % ����ط����ǽ���������
        v_NN0=v_NN;
        %             vorders=[vorders 0];
        %         else
    end
    
%     [search_stop]=stop_judge_special(x_new,y_new,theta_new,y_ini,ku,ITER+Iter);
    

    
    x=x_new;
    y=y_new;
    theta=theta_new;
    SS=[SS,[x;y;theta]];
    vorders=[vorders v_NN];
    %% TODO����λ��������α��ⷴ����⣿-1.�Ӵ���ɢ������;2.���ۺ������ǻ�������;3.��������
    
end

% ������һ��֮��


% end
[reward_stop,reward_detail]=RewardFunction(x_new,y_new,theta_new,[Action aangle],Risk0);
[if_success] = reachTarget(x,y,theta,ku);
end