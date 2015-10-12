close all
clear all

k=2
D=.4
m=10
A=[0 1;-k/m -D/m]
x=[1 0]'

sim_length=1000;

xlog=zeros(2,sim_length);
sensor1=zeros(1,sim_length);
sensor2=zeros(1,sim_length);
time_step=.05;

h_matrix=[1 0];

q_matrix=.04*eye(2);
r_matrix=10;
estimate=zeros(2,sim_length);
p_matrix=0*eye(2);
sys = ss(A,zeros(2,1),zeros(1,2),0)
discerete_a=c2d(sys, time_step)
discerete_a = discerete_a.a


for i=1:sim_length
% model
    x=discerete_a*x;
    % x=xdot*time_step+x;
    xlog(:,i)=x;
    sensor1(i)=h_matrix*x+.4*randn();
% kalman filter
    if i>1
        half_estimate=discerete_a*estimate(:,i-1);
        half_p_matrix=discerete_a*p_matrix*discerete_a'+q_matrix;
        y=sensor1(i)-h_matrix*half_estimate;
        s_matrix=h_matrix*half_p_matrix*h_matrix' + r_matrix;
        k_matrix = half_p_matrix*h_matrix'*inv(s_matrix);
        estimate(:,i)=half_estimate+k_matrix*y;
        p_matrix = (eye(2) - k_matrix*h_matrix)*half_p_matrix;
    end
end






figure()
plot(xlog(1,:))
hold on
plot(estimate(1,:),'ro')
plot(sensor1,'go')