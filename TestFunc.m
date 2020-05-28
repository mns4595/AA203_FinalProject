clear all, clc

tic

x1=[4;4]; x2=[5;6]; x3=[7;3]; x4=[10;4]; x5=[20;4];
WayPoints=[x1,x2,x3,x4,x5]';
Times=[2,.4,.5,3];
polyOrder=9;

[xCoeff,yCoeff,xTraj,yTraj,cost]=TrajOpt(WayPoints,Times,polyOrder);

figure
for i=1:length(WayPoints)-1
    plot(xTraj{i},yTraj{i})
    hold on
end
scatter(WayPoints(:,1),WayPoints(:,2))

toc





%Just to make sure coefficients were being outputed correctly as well as
%trajectories


% clear xTraj yTraj
% 
% 
% dt=0.01;
% T=cell(1,length(WayPoints)-1);
% xTraj=cell(1,length(WayPoints)-1);
% yTraj=cell(1,length(WayPoints)-1);
% 
% for i=1:length(WayPoints)-1
%     T{i}=0:dt:Times(i);
%     x=zeros(1,length(T{i}));
%     y=zeros(1,length(T{i}));
%     idx=2*(i-1)+1;
%     for n=1:length(T{i})
%         x(n)=0;
%         y(n)=0;
%         order=polyOrder;
%         for j=1:polyOrder+1
%             x(n)=x(n)+xCoeff{i}(j)*T{i}(n)^order;
%             y(n)=y(n)+yCoeff{i}(j)*T{i}(n)^order;
%             order=order-1;
%         end
%     end
%     xTraj{i}=x;
%     yTraj{i}=y;
% end
% 
% figure
% for i=1:length(WayPoints)-1
%     plot(xTraj{i},yTraj{i})
%     hold on
% end
% scatter(WayPoints(:,1),WayPoints(:,2))




