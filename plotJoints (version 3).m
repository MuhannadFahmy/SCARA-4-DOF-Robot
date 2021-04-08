function plotJoints(filename)
data = readmatrix(filename);
% Joint 1
figure(1)
graph1 = subplot(4,1,1);
plot(graph1,data(:,1),data(:,2),'-mx',data(:,1),data(:,3),'-go')
axis padded
axes('Position',[0.15, 0.5, 0.8, 0.45]);
set(graph1,'xticklabel',[],'xtick',[])
set(graph1,'position',[.1 .75 .8 .2])
ylabel(graph1,strcat('\Theta_1'))
title(graph1,strcat('\Theta_1, \Theta_2, \Theta_3, \Theta_4 vs time'))
legend(graph1,'planned','current')
% Joint 2
graph2 = subplot(4,1,2);
plot(graph2,data(:,1),data(:,4),'-mx',data(:,1),data(:,5),'-go')
axis padded
axes('Position',[0.15, 0.05, 0.8, 0.45]);
set(graph2,'xticklabel',[],'xtick',[])
set(graph2,'position',[.1 .525 .8 .2])
ylabel(graph2,'\Theta_2')
legend(graph2,'planned','current')
% Joint 3
graph3 = subplot(4,1,3);
plot(graph3,data(:,1),data(:,6),'-mx',data(:,1),data(:,7),'-go')
axis padded
axes('Position',[0.15, 0.05, 0.8, 0.45]);
set(graph3,'xticklabel',[],'xtick',[])
set(graph3,'position',[.1 .3 .8 .2])
ylabel(graph3,'h_3')
legend(graph3,'planned','current')
% Joint 4
graph4 = subplot(4,1,4);
plot(graph4,data(:,1),data(:,8),'-mx',data(:,1),data(:,9),'-go')
axis padded
set(graph4,'position',[.1 .075 .8 .2])
xlabel(graph4,'time (s)')
ylabel(graph4,'\Theta_4')
legend(graph4,'planned','current')
linkaxes([graph1 graph2 graph3 graph4],'x')
% Path graph
figure(2)
plot(data(:,10),data(:,11))
xlabel('x')
ylabel('y')
title('Path graph')
axis padded
end