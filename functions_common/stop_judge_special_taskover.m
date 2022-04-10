function [search_stop]=stop_judge_special_taskover(x,y,theta,y_ini,ku,action_length,cx,aaction)
%用于executeEpisode判定是否任务终止
search_stop=0;
%% 1.超时停止
if action_length>=30/0.05;%25秒 这个时间是会影响最后的结果的。 ==TODO== HowSun
    % action_length 是已经执行的时间长度
    search_stop=1;
else
    %% 2.高于起始位姿停止
    if y>(y_ini+0.2)
        search_stop=1;
    end
    %% 3.碰撞
    if collision(x,y,theta,ku)>0
        search_stop=2;
    end
    %% 4.成功：无碰撞且车已经正了
%     if collision(x,y,theta,ku)==0 && action_length>100 && abs(theta)<1*pi/180
    if collision(x,y,theta,ku)==0 && theta<0.01*pi/180 && y < 0%
        search_stop=3;
    end
    % % % [search_stop]=stop_judge_special(x,y,theta,y_ini,ku,action_length);
    %% 5.指令执行完毕
%     if cx==length(aaction)
%         search_stop=1;
%     end

    if x<-ku + 0.05 + 0.544
        search_stop = 5;
    end
end
end