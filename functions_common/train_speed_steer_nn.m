function train_speed_steer_nn(train_inputs1, train_outputs1, train_outputs2)

global SteerNN ;
global SpeedNN ;

%% ---------训练转角部分
% 输入特征应该包括开始时的点的state，即 x y thtea 转角 车速
data_out = zeros(9, size(train_outputs1,2));
for i = 1:size(train_outputs1,2)
    j = round(train_outputs1(i)/5)+5;
    if j>9
        j = 9;
    end
    if j<1
        j = 1;
    end
    data_out(j,i) = 1;
end
gpuDevice(1);
SteerNN_best = SteerNN;
perf_min = 100;
for kk = 1:1

    SteerNN=train(SteerNN_best, train_inputs1(:,1:end), data_out(:,1:end),'useGPU','yes');

    y = SteerNN(train_inputs1);
    perf = perform(SteerNN,data_out,y)
    classes = vec2ind(y);
    if perf < perf_min
        perf_min = perf;
        SteerNN_best = SteerNN;
    end
end

% save SteerNN_201110.mat SteerNN_best
genFunction(SteerNN_best,'steerNN_1110.m','MatrixOnly','yes');

%% ----------训练车速部分
data_out = zeros(3, size(train_outputs2,2));
for i = 1:size(train_outputs2,2)
    if train_outputs2(i)>0.005
        data_out(3,i) = 1;
    elseif train_outputs2(i)<-0.005
        data_out(1,i) = 1;
    else
        data_out(2,i) = 1;
    end
    
end
gpuDevice(1);
SpeedNN_best = SpeedNN;
perf_min = 100;
for kk = 1:1
    SpeedNN=train(SpeedNN_best, train_inputs1(:,1:end), data_out(:,1:end),'useGPU','yes');
    y = SpeedNN(train_inputs1);
    perf = perform(SpeedNN,data_out,y)
    classes = vec2ind(y);
    if perf < perf_min
        perf_min = perf;
        SpeedNN_best = SpeedNN;
    end
end

% save SpeedNN_201110.mat SpeedNN_best
genFunction(SpeedNN_best,'speedNN_1110.m','MatrixOnly','yes');



