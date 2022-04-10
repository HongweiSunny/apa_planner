function  [percentErrors]=trainNN(examples,PI,angle_real)
x0=examples(1,:);
y0=examples(2,:);
navigate_angle=examples(3,:);
Kuwei=examples(4,:);
L=length(x0);
target1=zeros(9,L);
for i=1:1:L
    j=round(PI(i)/5)+5;
    if j>9
        j=9;
    elseif j<1
        j=1;
    end
    target1(j,i)=1;
end
x= [x0;y0;navigate_angle;angle_real];
t=target1;
trainFcn = 'trainscg';
hiddenLayerSize = [25,25];
net = patternnet(hiddenLayerSize,trainFcn);
net.input.processFcns = {'mapminmax'};
net.output.processFcns ={};
net.performFcn = 'crossentropy';  
%%
net.layers{1}.transferFcn='tansig';
net.layers{2}.transferFcn='tansig';
%%
net.trainParam.epochs=2000;
% net.performParam.regularization = 0.1;
net.divideFcn = 'dividerand';  % Divide data randomly
net.divideMode = 'none';  % Divide up every sample
net.divideParam.trainRatio = 80/100;
net.divideParam.valRatio = 10/100;
net.divideParam.testRatio = 10/100;
% Choose a Performance Function
net.plotFcns = {'plotperform','plottrainstate','ploterrhist','plotconfusion', 'plotroc'};
% Train the Network
% [net,tr]= train(net,x1,t1);
% net.output.processFcns = {'mapminmax'};%'removeconstantrows',
net.trainParam.showWindow=false;
gpuDevice(1)
net=train(net,x,t,'useGPU','yes');
% net=train(net,x,t,'useParallel','yes','useGPU','only','showResources','yes');
% Test the Network
y = net(x);
e = gsubtract(t,y);
performance = perform(net,t,y);
tind = vec2ind(t);
yind = vec2ind(y);
percentErrors = sum(tind ~= yind)/numel(tind);
PN_net=net;
save PN_net.mat PN_net
genFunction(net,'myNN','MatrixOnly','yes');
end

