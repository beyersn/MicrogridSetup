%Kalman filter example

%This code is used to model a cannonball firing at a fixed angle.
clear all
close all
clc

ts=0.1;
iterations=145;
noiselevel=30;
muzzel_vel=100;
angle = pi/4;

c = Cannon(ts, noiselevel);

%State vector initial estimate
x0 = [0;
      muzzel_vel*cos(angle);
      500;
      muzzel_vel*sin(angle)];
  
%Initial estimate of state covariance
P0 = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];

%State transition matrix
A = [1 ts 0 0;
     0 1  0 0;
     0 0  1 ts;
     0 0  0 1];

%Control matrix
B = [0 0 0 0;
     0 0 0 0;
     0 0 1 0;
     0 0 0 1];

%Observation matrix
H = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];

%Process error covariance
Q = [0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0];
 
%Measurement error covariance
R = [.2 0 0 0;
     0 .2 0 0;
     0 0 .2 0;
     0 0 0 .2];

%Control vector (does not change in this case)
u_n = [0;
     0;
     0.5*-9.81*ts^2;
     -9.81*ts];
 
%Create Kalman filter object
kf = KalmanFilterLinear(A,B,H,Q,R,x0,P0);
    
x = [];
y = [];
nx = [];
ny = [];
kx = [];
ky = [];

%For loop to run through the model
for i = 0:iterations
   %get the true path
   x = [x,c.getXPos()];
   y = [y,c.getYPos()];
   %get the readings from the sensor
   newestX = c.getXWNoise();
   newestY = c.getYWNoise();
   nx = [nx,newestX];
   ny = [ny,newestY];
   %get the estimates from the filter
   kt=kf.getCurrentState();
   kx = [kx,kt(1)];
   ky = [ky,kt(3)];
   %propagate both the filter and the model
   c.step();
   z_n = [newestX;
          c.getXVel();
          newestY;
          c.getYVel];
   kf.step(u_n,z_n);  
end

figure(1)
plot(x,y,'- b',nx,ny,': g',kx,ky,'-- r')
xlabel('X position')
ylabel('Y position')
title('Measurement of a Cannonball in Flight')
legend('true','measured','kalman')




