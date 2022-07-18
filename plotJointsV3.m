function plotJoints(filename)
data = readmatrix("outputValues.csv");
% Joint 1
figure(1)
graph1 = subplot(4,1,1);
plot(graph1,data(:,1),data(:,2),'-','color','#A2142F','lineWidth',1)
hold on
plot(graph1,data(:,1),data(:,3),'-','color','#0072BD','lineWidth',1)
axis padded
axes('Position',[0.15, 0.5, 0.8, 0.45]);
set(graph1,'xticklabel',[],'xtick',[])
set(graph1,'position',[.1 .75 .8 .2])
ylabel(graph1,strcat('\Theta_1'))
title(graph1,strcat('\Theta_1, \Theta_2, \Theta_3, \Theta_4 vs time'))
legend(graph1,'planned','current')
% Joint 2
graph2 = subplot(4,1,2);
plot(graph2,data(:,1),data(:,4),'-','color','#A2142F','lineWidth',1)
hold on
plot(graph2,data(:,1),data(:,5),'-','color','#0072BD','lineWidth',1)
axis padded
axes('Position',[0.15, 0.05, 0.8, 0.45]);
set(graph2,'xticklabel',[],'xtick',[])
set(graph2,'position',[.1 .525 .8 .2])
ylabel(graph2,'\Theta_2')
legend(graph2,'planned','current')
% Joint 3
graph3 = subplot(4,1,3);
plot(graph3,data(:,1),data(:,6),'-','color','#A2142F','lineWidth',1)
hold on
plot(graph3,data(:,1),data(:,7),'-','color','#0072BD','lineWidth',1)
axis padded
axes('Position',[0.15, 0.05, 0.8, 0.45]);
set(graph3,'xticklabel',[],'xtick',[])
set(graph3,'position',[.1 .3 .8 .2])
ylabel(graph3,'h_3')
legend(graph3,'planned','current')
% Joint 4
graph4 = subplot(4,1,4);
plot(graph4,data(:,1),data(:,8),'-','color','#A2142F','lineWidth',1)
hold on
plot(graph4,data(:,1),data(:,9),'-','color','#0072BD','lineWidth',1)
axis padded
set(graph4,'position',[.1 .075 .8 .2])
xlabel(graph4,'time (s)')
ylabel(graph4,'\Theta_4')
legend(graph4,'planned','current')
linkaxes([graph1 graph2 graph3 graph4],'x')
% Path graph
figure(2)
%% planned path
c1 = cos(deg2rad(data(:,2)));
s1 = sin(deg2rad(data(:,2)));
c12 = cos(deg2rad(data(:,2))+deg2rad(data(:,4)));
s12 = sin(deg2rad(data(:,2))+deg2rad(data(:,4)));
L1 =195;
L2 = 142;
x = c1 * L1 + c12* L2;
y = s1 * L1 + s12* L2;
%% executed path
cc1 = cos(deg2rad(data(:,3)));
sc1 = sin(deg2rad(data(:,3)));
cc12 = cos(deg2rad(data(:,3))+deg2rad(data(:,5)));
sc12 = sin(deg2rad(data(:,3))+deg2rad(data(:,5)));
xc = cc1 * L1 + cc12* L2;
yc = sc1 * L1 + sc12* L2;

plot(x,y, '-','color','#A2142F','lineWidth',1)
legend('planned')
hold on
plot(xc,yc, '-','color','#0072BD','lineWidth',1)
hold off
legend('planned','executed')

xlabel('x')
ylabel('y')
title('Path graph')
axis padded
end