function train_speed_steer_nn(train_inputs1, train_outputs1, train_outputs2)

global VA_NN;

% --------- 数据准备部分
% 输入特征应该包括开始时的点的state，即 x y thtea 转角 车速
data_out = zeros(15, size(train_outputs1,2));
a_ind_all = [];
v_ind_all = [];
for i = 1:size(train_outputs1,2)
    a_ind = train_outputs1(i)/10 + 3;
    v_ind = round( train_outputs2(i)*1000/(15) + 2);
    a_ind_all = [a_ind_all a_ind];
    v_ind_all = [v_ind_all v_ind];
    
    
    j = (v_ind-1)*5 + a_ind;
    
    data_out(int32(j),int32(i)) = 1;
end

% ----------
gpuDevice(1);
VA_NN = train(VA_NN, train_inputs1(:,1:end), data_out(:,1:end),'useGPU','yes');

% ----------
y = VA_NN(train_inputs1);
perf = perform(VA_NN,data_out,y)
% classes = vec2ind(y);


% ---------
save('.\function_NN_20p_457NLP\VA_NN_210310.mat', 'VA_NN')
genFunction(VA_NN,'.\function_NN_20p_457NLP\VA_NN_201126.m','MatrixOnly','yes');




