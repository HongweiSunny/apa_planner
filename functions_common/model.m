function newPose = model(oldPose, steerOrder, veloOrder)
    newPose = zeros(3,1);
    dt = 0.05; 
    x = oldPose(1);
    y = oldPose(2);
    theta = oldPose(3);
    chuan=15.88;
    if steerOrder>530
        steerOrder=530;
    end
    if steerOrder<-530
        steerOrder=-530;
    end
    fai = steerOrder/chuan*pi/180;
    L = 2.305;
    newPose(1,1) = x + veloOrder * dt *cos(theta);
    newPose(2,1) = y + veloOrder * dt *sin(theta);
    newPose(3,1) = theta + veloOrder.*tan(fai)/L * dt;
end
%     theta_dot = veloReal.*tan(fai)/L*1.0; % 弧度制？
    % carsim的标定传动比
%     if steerOrder > 480
%         chuan = 15.8549;
%     elseif steerOrder > 380
%         chuan = 15.8861;
%     elseif steerOrder > 280
%         chuan = 15.9079;
%     elseif steerOrder > 180
%         chuan = 15.8766;  
%     elseif steerOrder > -180
%         chuan = 15.8780; 
%     elseif steerOrder > -280
%         chuan = 15.8724; 
%     elseif steerOrder > -380
%         chuan = 15.9082;  
%     elseif steerOrder > -480
%         chuan = 15.8848; 
%     else
%         chuan = 15.8555;
%     end
        % 转角限制幅度
%     if steerOrder>530
%         steerOrder=530;
%     end
%     if steerOrder<-530
%         steerOrder=-530;
%     end
    % ===== 实车标定的传动比
%     global qulv_angle;
% %     qulv_angle = [-530,-0.278708052005189;-529.403416097337,-0.278708052005189;-519.392917847036,-0.273635595615637;-509.184952507891,-0.264708402248353;-499.186885519097,-0.257748086059090;-489.417513747716,-0.251634875279504;-479.981919680033,-0.244667419971442;-470.056290618225,-0.238658026261239;-459.441243126153,-0.231496493922220;-449.395450758211,-0.224516028286485;-439.526795534105,-0.217174112664179;-429.294600899862,-0.210334259691193;-419.476031995439,-0.205040526323135;-409.454363662320,-0.197569845888149;-399.407656049125,-0.192086799480774;-379.200011109882,-0.181280078391441;-360.258837906866,-0.170442225673418;-339.855782690839,-0.159756971454636;-320.357914208570,-0.150920562687936;-301.096175318709,-0.140981158175243;-281.165986167815,-0.129897072138430;-260.887476043668,-0.120170068188248;-240.503433104440,-0.109856708532056;-220.348476768253,-0.100563181897879;-199.878148123466,-0.0907325929457492;-180.841157942117,-0.0827526124045591;-160.756872156382,-0.0732880425133664;-140.422648867534,-0.0639216005648562;-120.777866106672,-0.0546953978500204;-99.7134116617973,-0.0447428288259421;-90.1363539778653,-0.0405074073189122;-79.9152552965462,-0.0355650394488391;-70.6032310480520,-0.0311754726567704;-60.6629710643218,-0.0268049808245790;-50.8398600087491,-0.0220377347056903;-40.9688269483159,-0.0175749593521804;-20.3973001687406,-0.00876756397076694;29.5845947128329,0.0134046408570499;40.0526154615329,0.0181476675248422;50.2929191925617,0.0229290247181393;59.8038560090050,0.0271811238224660;70.4048934441791,0.0319650437785120;78.9034997812627,0.0357502640028456;89.0832885444426,0.0404815671315173;99.2396412724431,0.0452606446047071;119.439323033858,0.0535590187016379;139.526228688574,0.0629926426524907;160.132718364083,0.0724902536949043;179.660756962130,0.0814189320233299;200.350622468889,0.0907808968706499;219.861869208678,0.100517242560865;238.443670421908,0.109305974284733;260.152628947593,0.119300904744523;279.841646529481,0.130223287918236;299.764194650426,0.140322038934831;319.792317306873,0.150665288962523;339.380346628132,0.161083266517350;359.015131651987,0.171415410955020;378.375902677455,0.181341579814146;398.691029852875,0.192046097862144;408.504027995986,0.197210240531146;418.474460791302,0.205009336737059;428.556534780753,0.210969510732340;438.539310114968,0.216544894249541;448.674570904842,0.223218964714015;458.562689551755,0.230216967043013;468.643092817870,0.236996350637561;478.792551241457,0.242054676361554;489.091168138671,0.250077584163225;498.865239126814,0.256568481270797;508.882802866156,0.264490317894075;518.445159140106,0.269450805284397;528.749658390269,0.277721424239893;530,0.277721424239893];
%     qulv = interp1(qulv_angle(:,1), qulv_angle(:,2), steerOrder);
%         
%     % 传动比
%     if abs(steerOrder)==530
%         chuan=16.2425;
%     else
%         chuan=16.68;
%     end
        
%     % --更新位姿
%     % 前轮转角
% %     fai = -steerOrder/chuan*pi/180;  % 弧度制
%     L = 2.305;
%     x_dot = veloReal.*cos(theta)*1.0;
%     y_dot = veloReal.*sin(theta)*1.0;
% %     theta_dot = veloReal.*tan(fai)/L*1.0; % 弧度制？
%     theta_dot = -veloReal.* qulv; % 直接乘上曲率
%     x_new = x+x_dot*dt;
%     y_new = y+y_dot*dt;
%     theta_new=theta+theta_dot*dt;

        % --更新位姿
    % 前轮转角
%     fai = -steerOrder/chuan*pi/180;  % 弧度制
%     L = 2.305;
%     % 先固定一个delta_s
% %     delta_s = 0.05 * 0.4; % 假设最大车速0.4；
%     delta_s = 0.05*SpeedNN_out([oldPose; steerOrder]);
%     x_dot = -delta_s.*cos(theta);
%     y_dot = -delta_s.*sin(theta);
% %     theta_dot = veloReal.*tan(fai)/L*1.0; % 弧度制？
%     theta_dot = delta_s.* qulv; % 直接乘上曲率
%     x_new = x+x_dot;
%     y_new = y+y_dot;
%     theta_new=theta+theta_dot;
%     
%     veloOrder = delta_s;
%     if dist<=0.25
%         veloOrder = 0;
%     end
    
    % --新位姿输出
%     newPose = [x_new, y_new, theta_new]';
% end
% function veloOrder = get_velo_order_(iter, dt, dist)
%     % 最大加速度
%     a_max=0.3;
%     % 斜率 2.5s内达到最大加速度
%     xielv0=a_max/2.5; 
%     % 初始速度
% %     v0 = -0.2; 
%     v0 = -0.001; 
%     % 时间
%     Time = iter * dt;
%     % 计算当前速度指令
%     if Time<=2.5
%         % 1/2 a t
%         dv = 0.5*Time*(Time*xielv0);
%         veloOrder = v0-dv;
%     elseif Time>2.5 && Time<=5
%         dv = 0.5*(a_max+(a_max-xielv0*(Time-2.5)))*(Time-2.5);
%         veloOrder = v0-0.375-dv;
%     elseif Time>5 && Time <=7
%         veloOrder = -0.7510;
%     elseif Time>7 && Time <=10
%         veloOrder = -0.751 + 0.15*(Time - 7);
%     else
%         veloOrder = -0.3;
%     end
%     if dist<0.20
%         veloOrder = 0;
%     end
% %         veloOrder = 0;
% %     % 5s后看距离
% %     elseif dist<=1.5 && dist>=0.05
% %         veloOrder = -0.5621*dist-0.1069;
% %     elseif dist<0.05
% %         veloOrder = 0;
% %     else
% %         veloOrder = -0.751;
% %     end
% end
% function veloOrder = get_velo_order_bk(iter, dt, dist)
%     % 最大加速度
%     a_max=0.3;
%     % 斜率 2.5s内达到最大加速度
%     xielv0=a_max/2.5; 
%     % 初始速度
% %     v0 = -0.2; 
%     v0 = -0.001; 
%     % 时间
%     Time = iter * dt;
%     % 计算当前速度指令
%     if Time<=2.5
%         % 1/2 a t
%         dv = 0.5*Time*(Time*xielv0);
%         veloOrder = v0-dv;
%     elseif Time>2.5 && Time<=5
%         dv = 0.5*(a_max+(a_max-xielv0*(Time-2.5)))*(Time-2.5);
%         veloOrder = v0-0.375-dv;
%         
%     % 5s后看距离
%     elseif dist<=1.5 && dist>=0.05
%         veloOrder = -0.5621*dist-0.1069;
%     elseif dist<0.05
%         veloOrder = 0;
%     else
%         veloOrder = -0.751;
%     end
% end
% function veloOrder = get_velo_order_s(iter, x, xs, dist)
% % input: x dist
%     if x>=0
% %         xs = 1.5;
% %         x = 1.5:-0.1:0;
%         k = 0.8/xs; 
%         veloOrder = (x-xs) .* k - 0.2; % x>0的部分的增长斜率需要考虑下结合初始位姿的x的大小
%     elseif x>=-2
%         veloOrder = -1;
%     elseif x>=-4
%         veloOrder = -1 + (-2-x) * 0.4;
%     else
%         veloOrder = -0.2;
%     end
%     if dist<= 0.2
%         veloOrder = 0;
%     end
% end
% function veloOrder = get_velo_order_from_est(iter, dt, dist)
%     global vr_est_arr_me_filtered;
%     if iter > length(vr_est_arr_me_filtered)
%         iter = length(vr_est_arr_me_filtered);
%     end
%     veloOrder = vr_est_arr_me_filtered(iter);
%     if dist<=0.2
%         veloOrder = 0;
%     end
% end

