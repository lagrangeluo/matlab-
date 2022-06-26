% /**
%   ******************************************************************************
%   * @file    Matlab/库/luojunyu_1905_circlecircle.m
%   * @author  罗俊宇
%   * @version V2.3
%   * @date    2022.4
%   * @brief   机器人学机械手臂仿真实验
%   ******************************************************************************
%   * @attention 实现机械手臂的螺旋线运动轨迹
%   ******************************************************************************
%   *********************(C) COPYRIGHT 2022 NEUQRoboMaster************************
%   .......................... 革命尚未成功，同志仍需努力  .........................
% */  
clear all;
clc;
M=300;l=150;L=50;
per_R=500/720;%螺旋线半径变化范围
hd = pi/180; %一度对应的弧度
du=180/pi;  %一弧度对应的度数
TE3=[1 0 L;
     0 1 0;
     0 0 1];
theta=1:1:720;%绘图所用theta，不参与实际运算
for i=1:720
    phi(i)=(i-1)*hd; 
    R(i)=250+(i-1)*per_R;
      TE0=[cos(phi(i)-pi/2)     -sin(phi(i)-pi/2)       R(i)*cos(phi(i));
           sin(phi(i)-pi/2)      cos(phi(i)-pi/2)       R(i)*sin(phi(i));
           0                      0                     1];
                   
      T30=TE0*inv(TE3);
      theta1(i)=atan2(T30(2,3),T30(1,3));
      d(i)=sqrt(T30(1,3)*T30(1,3)+T30(2,3)*T30(2,3))-l;
      theta3(i)=atan2(T30(2,1),T30(1,1))-theta1(i);

      T30_f=[cos(theta1(i)+theta3(i))   -sin(theta1(i)+theta3(i))   (l+d(i))*cos(theta1(i));
             sin(theta1(i)+theta3(i))   cos(theta1(i)+theta3(i))    (l+d(i))*sin(theta1(i));
             0                          0                            1];

      TE0_f=T30_f*TE3;
      x_f(i)=TE0_f(1,3);
      y_f(i)=TE0_f(2,3);

end
figure(1);
subplot(2,2,1);
plot(theta,d);
title('d随角度变换图');
xlabel('/°');
ylabel('/mm');
axis([0 800 -80 -79],'equal');
grid on;

subplot(2,2,2);
plot(theta,theta3);
title('theta3随角度变换图');
xlabel('/°');
ylabel('/弧度');
axis([0 800 -4 6]);
grid on;

subplot(2,2,3);
plot(theta,theta1);
title('theta1随角度变换图');
xlabel('/°');
ylabel('/弧度');
axis([0 800 -4 6]);
grid on;

subplot(2,2,4);
plot(x_f,y_f);
title('末端执行器轨迹图');
grid on;
axis equal;

%运动仿真，电影制作
figure(2);
j=0;
for n1 = 1:4:720
    j=j+1;
    clf;
    x(1)=0;
    y(1)=0;
    x(2)=d(n1)*cos(theta1(n1));
    y(2)=d(n1)*sin(theta1(n1));
    x(3)=(d(n1)+l)*cos(theta1(n1));
    y(3)=(d(n1)+l)*sin(theta1(n1));
    x(4)=x_f(n1);
    y(4)=y_f(n1);
    i=1:2;
        plot(x(i), y(i),'LineWidth',3);
        hold on;
    i=2:4;
        plot(x(i), y(i));
        grid on;
        hold on;
    i=1:n1;
        plot(x_f(i), y_f(i));
        plot(x(1),y(1),'o')
        plot(x(2),y(2),'o')
        plot(x(3),y(3),'o')
        plot(x(4),y(4),'*')    
    title('机械臂运动模拟');
    xlabel('mm');
    ylabel('mm');
    axis([-500 500 -700 700],'equal');
    m(j)=getframe;

end
    movie(m,1);



