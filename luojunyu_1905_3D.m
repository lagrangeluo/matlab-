% /**
%   ******************************************************************************
%   * @file    Matlab/库/luojunyu_1905_3D.m
%   * @author  罗俊宇
%   * @version V1.0
%   * @date    2022.5
%   * @brief   机器人学机械手臂仿真实验
%   ******************************************************************************
%   * @attention 
%   ******************************************************************************
%   *********************(C) COPYRIGHT 2022 NEUQRoboMaster************************
%   .......................... 革命尚未成功，同志仍需努力  .........................
% */  
clear;
clc;
L0=15;L1=150;L2=20;L3=120;L4=20;LE=10;
hd = pi/180; %一度对应的弧度
du=180/pi;  %一弧度对应的度数
syms theta1 theta2 theta3 theta4 theta5 theta6;
theta=[theta1 theta2 theta3 theta4 theta5 theta6];
t=1:720;
TE6=[1 0 0 0;
      0 1 0 0;
      0 0 1 20;
      0 0 0 1];
DH=[0 0 0 0;
    pi/2 0 pi/2 0;
    0 L1 0 0;
    pi/2 L2 0 L3;
    -pi/2 0 0 0;
    pi/2 0 0 0;
    0 0 0 LE];
 for i=1:6
    if(i==1)
    T10=[cos(theta(i)) -sin(theta(i)) 0 DH(i,2);
     sin(theta(i))*cos(sym(DH(i,1))) cos(theta(i))*cos(sym(DH(i,1))) -sin(sym(DH(i,1))) -sin(sym(DH(i,1)))*DH(i,4);
     sin(theta(i))*sin(sym(DH(i,1))) cos(theta(i))*sin(sym(DH(i,1))) cos(sym(DH(i,1)))  cos(sym(DH(i,1)))*DH(i,4);
     0 0 0 1];
    end
    if(i==2)
    T21=[cos(theta(i)) -sin(theta(i)) 0 DH(i,2);
     sin(theta(i))*cos(sym(DH(i,1))) cos(theta(i))*cos(sym(DH(i,1))) -sin(sym(DH(i,1))) -sin(sym(DH(i,1)))*DH(i,4);
     sin(theta(i))*sin(sym(DH(i,1))) cos(theta(i))*sin(sym(DH(i,1))) cos(sym(DH(i,1)))  cos(sym(DH(i,1)))*DH(i,4);
     0 0 0 1];
    end
    if(i==3)
    T32=[cos(theta(i)) -sin(theta(i)) 0 DH(i,2);
     sin(theta(i))*cos(sym(DH(i,1))) cos(theta(i))*cos(sym(DH(i,1))) -sin(sym(DH(i,1))) -sin(sym(DH(i,1)))*DH(i,4);
     sin(theta(i))*sin(sym(DH(i,1))) cos(theta(i))*sin(sym(DH(i,1))) cos(sym(DH(i,1)))  cos(sym(DH(i,1)))*DH(i,4);
     0 0 0 1];
    end
    if(i==4)
    T43=[cos(theta(i)) -sin(theta(i)) 0 DH(i,2);
     sin(theta(i))*cos(sym(DH(i,1))) cos(theta(i))*cos(sym(DH(i,1))) -sin(sym(DH(i,1))) -sin(sym(DH(i,1)))*DH(i,4);
     sin(theta(i))*sin(sym(DH(i,1))) cos(theta(i))*sin(sym(DH(i,1))) cos(sym(DH(i,1)))  cos(sym(DH(i,1)))*DH(i,4);
     0 0 0 1];
    end    
    if(i==5)
    T54=[cos(theta(i)) -sin(theta(i)) 0 DH(i,2);
     sin(theta(i))*cos(sym(DH(i,1))) cos(theta(i))*cos(sym(DH(i,1))) -sin(sym(DH(i,1))) -sin(sym(DH(i,1)))*DH(i,4);
     sin(theta(i))*sin(sym(DH(i,1))) cos(theta(i))*sin(sym(DH(i,1))) cos(sym(DH(i,1)))  cos(sym(DH(i,1)))*DH(i,4);
     0 0 0 1];
    end
    if(i==6)
    T65=[cos(theta(i)) -sin(theta(i)) 0 DH(i,2);
     sin(theta(i))*cos(sym(DH(i,1))) cos(theta(i))*cos(sym(DH(i,1))) -sin(sym(DH(i,1))) -sin(sym(DH(i,1)))*DH(i,4);
     sin(theta(i))*sin(sym(DH(i,1))) cos(theta(i))*sin(sym(DH(i,1))) cos(sym(DH(i,1)))  cos(sym(DH(i,1)))*DH(i,4);
     0 0 0 1];
    end    
 end
 T60=T10*T21*T32*T43*T54*T65;
 alpha=zeros(1,720);
 x_i=zeros(1,720);
 y_i=zeros(1,720);
 z_i=zeros(1,720);
 theta1_f=zeros(1,720);
 theta2_f=zeros(1,720);
 theta3_f=zeros(1,720);
 theta4_f=zeros(1,720);
 theta5_f=zeros(1,720);
 theta6_f=zeros(1,720);
 x_f=zeros(1,720);
 y_f=zeros(1,720);
 z_f=zeros(1,720);
 for j=1:1:720
     alpha(j)=hd*j;
     x_i(j)=(10+0.2*j)*cos(alpha(j));
     y_i(j)=(10+0.2*j)*sin(alpha(j));
     z_i(j)=50+j*0.1;
    TE0=[1 0 0  x_i(j);
         0 -1 0 y_i(j);
         0 0 -1 z_i(j);
         0 0 0 1];
    theta1_yugu=atan2(TE0(2,4),TE0(1,4));
    %逆运动学求解
    f=T32*T43(:,4);
    g=T21*T32*T43(:,4);
%     k=[f(1) -f(2) f(1)*f(1)+f(2)*f(2)+f(3)*f(3) f(3)*cos(sym(pi/2))];
    theta3=fsolve(@(theta3)(120*cos(theta3) - 20*sin(theta3))^2 + (20*cos(theta3) + 120*sin(theta3) + 150)^2-(TE0(1,4)^2+TE0(2,4)^2+TE0(3,4)^2),[0]);
    theta2=fsolve(@(theta2)sin(theta2)*(20*cos(theta3) + 120*sin(theta3) + 150) - cos(theta2)*(120*cos(theta3) - 20*sin(theta3)) - TE0(3,4),[0]);
    theta1=fsolve(@(theta1)cos(theta1)*(150*cos(theta2) + 20*cos(theta2)*cos(theta3) + 120*cos(theta2)*sin(theta3) + 120*cos(theta3)*sin(theta2) - 20*sin(theta2)*sin(theta3)) - TE0(1,4),[theta1_yugu]);

    theta4=0;
    theta5=-theta3-theta2;
    theta6=0;
    theta_f=[theta1 theta2 theta3 theta4 theta5 theta6];
    theta1_f(j)=theta1;theta2_f(j)=theta2;theta3_f(j)=theta3;theta4_f(j)=theta4;theta5_f(j)=theta5;theta6_f(j)=theta6;
for i=1:6
    if(i==1)
    T10=[cos(theta_f(i)) -sin(theta_f(i)) 0 DH(i,2);
     sin(theta_f(i))*cos(sym(DH(i,1))) cos(theta_f(i))*cos(sym(DH(i,1))) -sin(sym(DH(i,1))) -sin(sym(DH(i,1)))*DH(i,4);
     sin(theta_f(i))*sin(sym(DH(i,1))) cos(theta_f(i))*sin(sym(DH(i,1))) cos(sym(DH(i,1)))  cos(sym(DH(i,1)))*DH(i,4);
     0 0 0 1];
    end
    if(i==2)
    T21=[cos(theta_f(i)) -sin(theta_f(2)) 0 DH(i,2);
     sin(theta_f(i))*cos(sym(DH(i,1))) cos(theta_f(i))*cos(sym(DH(i,1))) -sin(sym(DH(i,1))) -sin(sym(DH(i,1)))*DH(i,4);
     sin(theta_f(i))*sin(sym(DH(i,1))) cos(theta_f(i))*sin(sym(DH(i,1))) cos(sym(DH(i,1)))  cos(sym(DH(i,1)))*DH(i,4);
     0 0 0 1];
    end
    if(i==3)
    T32=[cos(theta_f(i)) -sin(theta_f(i)) 0 DH(i,2);
     sin(theta_f(i))*cos(sym(DH(i,1))) cos(theta_f(i))*cos(sym(DH(i,1))) -sin(sym(DH(i,1))) -sin(sym(DH(i,1)))*DH(i,4);
     sin(theta_f(i))*sin(sym(DH(i,1))) cos(theta_f(i))*sin(sym(DH(i,1))) cos(sym(DH(i,1)))  cos(sym(DH(i,1)))*DH(i,4);
     0 0 0 1];
    end
    if(i==4)
    T43=[cos(theta_f(i)) -sin(theta_f(i)) 0 DH(i,2);
     sin(theta_f(i))*cos(sym(DH(i,1))) cos(theta_f(i))*cos(sym(DH(i,1))) -sin(sym(DH(i,1))) -sin(sym(DH(i,1)))*DH(i,4);
     sin(theta_f(i))*sin(sym(DH(i,1))) cos(theta_f(i))*sin(sym(DH(i,1))) cos(sym(DH(i,1)))  cos(sym(DH(i,1)))*DH(i,4);
     0 0 0 1];
    end    
    if(i==5)
    T54=[cos(theta_f(i)) -sin(theta_f(i)) 0 DH(i,2);
     sin(theta_f(i))*cos(sym(DH(i,1))) cos(theta_f(i))*cos(sym(DH(i,1))) -sin(sym(DH(i,1))) -sin(sym(DH(i,1)))*DH(i,4);
     sin(theta_f(i))*sin(sym(DH(i,1))) cos(theta_f(i))*sin(sym(DH(i,1))) cos(sym(DH(i,1)))  cos(sym(DH(i,1)))*DH(i,4);
     0 0 0 1];
    end
    if(i==6)
    T65=[cos(theta_f(i)) -sin(theta_f(i)) 0 DH(i,2);
     sin(theta_f(i))*cos(sym(DH(i,1))) cos(theta_f(i))*cos(sym(DH(i,1))) -sin(sym(DH(i,1))) -sin(sym(DH(i,1)))*DH(i,4);
     sin(theta_f(i))*sin(sym(DH(i,1))) cos(theta_f(i))*sin(sym(DH(i,1))) cos(sym(DH(i,1)))  cos(sym(DH(i,1)))*DH(i,4);
     0 0 0 1];
    end    
end
T30=T10*T21*T32;
T40=T10*T21*T32*T43;
TE0=T10*T21*T32*T43*T54*T65*TE6;

figure(2);
x(1)=0;y(1)=0;z(1)=-20;
x(2)=0;y(2)=0;z(2)=0;
x(3)=T30(1,4);y(3)=T30(2,4);z(3)=T30(3,4);
x(4)=T40(1,4);y(4)=T40(2,4);z(4)=T40(3,4);
x(5)=TE0(1,4);y(5)=TE0(2,4);z(5)=TE0(3,4);
x_f(j)=TE0(1,4);y_f(j)=TE0(2,4);z_f(j)=TE0(3,4);
    i=1:2;
    plot3(x(i),y(i),z(i),'LineWidth',6);
    hold on;
    i=2:5;
    plot3(x(i),y(i),z(i),'LineWidth',1);
    hold on;
    k=1:1:j;
    plot3(x_f(k),y_f(k),z_f(k));
      plot3(x(2),y(2),z(2),'o')
      plot3(x(3),y(3),z(3),'o')
      plot3(x(4),y(4),z(4),'o')
      plot3(x(5),y(5),z(5),'*')    
    grid on;
    hold off;
    
    title('机械臂运动模拟');
    xlabel('X/mm');
    ylabel('Y/mm');
    zlabel('Z/mm')
    axis([-200 200 -200 200 -20 400]);
 end

figure(1);
subplot(2,3,1);
plot(t,theta1_f);
title('theta1变换图');
xlabel('/t');
ylabel('/rad');
grid on;
hold on;

subplot(2,3,2);
plot(t,theta2_f);
title('theta2变换图');
xlabel('/t');
ylabel('/rad');
grid on;
hold on;

subplot(2,3,3);
plot(t,theta3_f);
title('theta3变换图');
xlabel('/t');
ylabel('/rad');
grid on;
hold on;

subplot(2,3,4);
plot(t,theta4_f);
title('theta4变换图');
xlabel('/t');
ylabel('/rad');
grid on;
hold on;

subplot(2,3,5);
plot(t,theta5_f);
title('theta5变换图');
xlabel('/t');
ylabel('/rad');
grid on;
hold on;

subplot(2,3,6);
plot(t,theta6_f);
title('theta6变换图');
xlabel('/t');
ylabel('/rad');
grid on;
hold on;


 



