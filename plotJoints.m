function plotJoints(joint, num)
figure(1)
graph1 = subplot(3,1,1);
plot(graph1,joint(:,1),joint(:,2))
xlabel(graph1,'time(s)')
ylabel(graph1,strcat('\Theta',num))
title(graph1,strcat('\Theta_',num2str(num),' vs time'))
graph2 = subplot(3,1,2);
plot(graph2,joint(:,1),joint(:,3))
xlabel(graph2,'time(s)')
ylabel(graph2,'Velocity')
title(graph2,'Velocity vs time')
graph3 = subplot(3,1,3);
plot(graph3,joint(:,1),joint(:,4))
xlabel(graph3,'time(s)')
ylabel(graph3,'Accelaration')
title(graph3,'Acceleration vs time')
end