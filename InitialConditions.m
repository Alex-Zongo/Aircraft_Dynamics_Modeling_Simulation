 %Initialize constants for the RCAM simulation
clear
clc
close all

%% Define constants
x0 = [85;               %approx 165 knots
    0;
    0;
    0;
    0;
    0;
    0;
    0.1;                %approx 5.73 deg
    0];

u = [0;
    -0.1;               %approx 5.73 deg
    0;
    0.08;               %recall minimum for throttles are 05*pi/180 = 0.0087
    0.08];

TF=60;

pNED=[0;
      0;
      -1500];         % Aircraft NED Position in the Earth frame  
  
% eccentricity and semi-major axis for Meridian and prime vertical radi of curvature 
a=6378137.0;          %semi-major axis (m)
e=0.081819190842622;  %eccentricity

% initial latitude and longitude of Madrid, Spain
longitude0 = deg2rad(-3.70390);
latitude0 = deg2rad(40.416775);

 % Define min and max controls
u1min=-25*pi/180;
u1max=25*pi/180;

u2min=-25*pi/180;
u2max=10*pi/180;

u3min=-30*pi/180;
u3max=30*pi/180;

u4min=0.5*pi/180;
u4max=10*pi/180;

u5min=0.5*pi/180;
u5max=10*pi/180;



%% Run model
sim('simulink_aircraft_simulation.slx')

%% Plot the results
% t=simX.Time;
% 
% u1=simU.Data(:,1);
% u2=simU.Data(:,2);
% u3=simU.Data(:,3);
% u4=simU.Data(:,4);
% u5=simU.Data(:,5);   
% 
% x1=simX.Data(:,1);
% x2=simX.Data(:,2);
% x3=simX.Data(:,3);
% x4=simX.Data(:,4);
% x5=simX.Data(:,5);
% x6=simX.Data(:,6);
% x7=simX.Data(:,7);
% x8=simX.Data(:,8);
% x9=simX.Data(:,9);
% % x10=out.simX.Data(:,10);
% % x11=out.simX.Data(:,11);
% % x12=out.simX.Data(:,12);
% 
% %plot the control inputs
% figure
% subplot(5,1,1)
% plot(t, u1)
% legend('aileron = u_1')
% grid on
% 
% subplot(5,1,2)
% plot(t, u2)
% legend('stabilizer = u_2')
% grid on
% 
% subplot(5,1,3)
% plot(t, u3)
% legend('rudder = u_3')
% grid on
% 
% subplot(5,1,4)
% plot(t, u4)
% legend('throttle1 = u_4')
% grid on
% 
% subplot(5,1,5)
% plot(t, u5)
% legend('throttle2 = u_5')
% grid on
% 
% %plot the states
% figure
% %u,v,w
% subplot(3,3,1)
% plot(t, x1)
% legend('u = x_1')
% grid on
% 
% subplot(3,3,4)
% plot(t, x2)
% legend('v = x_2')
% grid on
% 
% subplot(3,3,7)
% plot(t, x3)
% legend('w = x_3')
% grid on
% 
% %p,q,r
% subplot(3,3,2)
% plot(t, x4)
% legend('p = x_4')
% grid on
% 
% subplot(3,3,5)
% plot(t, x5)
% legend('q = x_5')
% grid on
% 
% subplot(3,3,8)
% plot(t, x6)
% legend('r = x_6')
% grid on
% 
% %phi,theta,psi
% subplot(3,3,3)
% plot(t, x7)
% legend('phi = x_7')
% grid on
% 
% subplot(3,3,6)
% plot(t, x8)
% legend('theta = x_8')
% grid on
% 
% subplot(3,3,9)
% plot(t, x9)
% legend('psi = x_9')
% grid on