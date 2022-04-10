function [search_stop]=stop_judge_special(x,y,theta,y_ini,ku,action_length)%
%用于DEFAULTPOLICY判定是否应该停止仿真
search_stop=0;
%% 1.超时停止
if action_length>=30/0.05;%25秒
    search_stop=1;
else
    %% 2.高于起始位姿停止
    if y>(y_ini+0.2)
        search_stop=1;
    end
    %% 3.低于目标位姿停止
    if y<-2+1.544/2
        search_stop=2;
    end
    %% 4.达到目标停止
    if reachTarget(x,y,theta,ku)>0
        search_stop=3;
    end
    %% 5.碰撞停止（TODO：是否有更好办法？）
    if collision(x,y,theta,ku)>0
        search_stop=4;
    end
    %% 5. 把前面的碰撞检测改成直接利用X坐标
    if x<-ku + 0.05 + 0.544
        search_stop = 5;
    end
end
end