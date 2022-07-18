
%% part a)
data = readmatrix("1a.csv");
graph1 = subplot(3,1,1);
plot(graph1,data(:,1),data(:,2),'-','color','#A2142F','lineWidth',1)
axis padded
hold on;
fplot(graph1,thetaPloy,'-','color','#0072BD','lineWidth',1)
plot(t,theta)
hold off
set(graph1,'xticklabel',[],'xtick',[])
ylabel(graph1,strcat('Position'))
xlabel(graph1,'time (s)')
title(graph1,strcat('Position, Velocity, Acceleration, vs time'))
legend("cubicSpline","PolyFitting","parabolicbending")
% velcoity
graph2 = subplot(3,1,2);
plot(graph2,data(:,1),data(:,3),'-','color','#A2142F','lineWidth',1)
hold on;
fplot(graph2,DthetaPoly,'-','color','#0072BD','lineWidth',1)
plot(t(1:499),dtheta)
hold off
axis padded
set(graph2,'xticklabel',[],'xtick',[])
ylabel(graph2,strcat('Velocity'))
xlabel(graph2,'time (s)')
% Acceleration
graph3 = subplot(3,1,3);
plot(graph3,data(:,1),data(:,4),'-','color','#A2142F','lineWidth',1)
hold on;
fplot(graph3,DDthetaPoly,'-','color','#0072BD','lineWidth',1)
plot(t(1:498),ddtheta)
hold off
axis padded
set(graph3,'xticklabel',[],'xtick',[])
ylabel(graph3,strcat('Acceleration'))
linkaxes([graph1 graph2 graph3],'x')
xlabel(graph3,'time (s)')


%% part b

syms a0 a1 a2 a3 a4

eqn1 = a0 == 0;
eqn2 = a1 + 5*a2 + 25*a3 + 125*a4 == 12;
eqn3 = 10*a1 + 100*a2 + 1000*a3 + 10000*a4 ==30;
eqn4 = a1 ==0;
eqn5 = 20*a2 + 300*a3 + 4000*a4;

[A,B] = equationsToMatrix([eqn1,eqn2,eqn3,eqn4,eqn5],[a0,a1,a2,a3,a4]);
X = linsolve(A,B);

syms t 
thetaPloy = X(1) + X(2)*t + X(3)*t^2 + X(4)*t^3 + X(5)*t^4 ;
DthetaPoly = diff(thetaPloy)
DDthetaPoly = diff(thetaPloy,2)

plotf(DthetaPoly)

%% part c Parapolic Bending

t0= linspace(0,0.246,100);
t01= linspace(0.246,4.5705+0.246,100);
t1= linspace(4.5705+0.246,4.5705+0.246+0.367,100);
t12= linspace(4.5705+0.246+0.367,10-0.1214,100);
t2= linspace(10-0.1214,10,100);
t= [t0 t01 t1 t12 t2];

theta0=0.5*50*t0.^2;
theta01=theta0(end)+ 12.3*(t01-t0(end));
theta1=theta01(end)+0.8419+ -0.5*50*(t1-(t01(end)+0.367/2)).^2;
theta12=theta1(end)-(6.149*(t12-t1(end)));
theta2=theta12(end) + 0.5*50*(t2-t12(end)).^2;
theta = [theta0 theta01 theta1 theta12 theta2];
dtheta = diff(theta,1);
ddtheta = diff(theta,2);

%% Q2,b

%starting position

theta1(1) = deg2rad(130.897);
theta2(1)= deg2rad(132.454);
L1 =1;
L2 =1;
t=0;

velTheta1(1)=0;
velTheta2(1)=0;
while t <= 5
 
    s1= sin(theta1(end));
    s2= sin(theta2(end));
    s12= sin(theta1(end)+theta2(end));
    c1= cos(theta1(end));
    c2= cos(theta2(end));
    c12= cos(theta1(end)+theta2(end));
    Jinv=(1/(L1*L2*s2))*[L2*c12 L2 * s12; -L1*c1-L2*c12 -L1*s1-L2*s12];
    V= [0 ; -0.0064]; %per 20 ms
    dTheta= Jinv*V;
    velTheta1(end+1) = rad2deg(dTheta(1));
    velTheta2(end+1) =rad2deg(dTheta(2));
    theta1(end+1) = theta1(end)+dTheta(1);
    theta2(end+1)= theta1(end)+dTheta(2);
    t=t+0.02;
end

for i = 1:252
    theta1(i)= rad2deg(theta1(i));
    theta2(i)= rad2deg(theta2(i))
end

 
 tt= linspace(0,5,252);
 plot(tt,theta1,'color','#A2142F');
 hold on 
 plot(tt,theta2,'color','#0072BD');
 xlabel('time (s)');
    ylabel('\Theta (Deg)');

  plot(tt,velTheta1,'color','#A2142F'); 
 plot(tt,velTheta2,'color','#0072BD');
  legend("theta1","theta2","Dtheta1","Dtheta2")
 clear all;
 
 
%% 7.19
syms t;
theta(t)= 10 + 5*t + 70*t^2 - 45*t^3;
t = [0 1];
subs(theta(t))

Df = diff(theta);

Df0 = Df(0)
Df1 = Df(1)

DDf = diff(Df)

DDf0 = DDf(0)
DDf1 = DDf(1)
