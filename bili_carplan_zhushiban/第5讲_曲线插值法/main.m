% 曲线插值法
% 作者：Ally
% 日期：2021/1/16
clc
clear
close all

%% 场景定义
% 换道场景路段与车辆相关参数的定义
d = 3.5;          % 道路标准宽度
len_line = 30;    % 直线段长度
W = 1.75;         % 车宽
L = 4.7;          % 车长
x1 = 20;          %1号车x坐标 蓝车

% 车辆换道初始状态与终点期望状态
t0 = 0;
t1 = 3;% 秒
state_t0 = [0, -d/2; 5, 0; 0, 0];  % x,y; vx,vy; ax,ay 纵向，横向
state_t1 = [20, d/2; 5, 0; 0, 0];
x2 = state_t0(1); % 2号车 黄车

%% 画场景示意图
figure(1)
% 画灰色路面图
GreyZone = [-5,-d-0.5; -5,d+0.5; len_line,d+0.5; len_line,-d-0.5];
fill(GreyZone(:,1),GreyZone(:,2),[0.5 0.5 0.5]);
hold on

% 画小车
fill([x1,x1,x1+L,x1+L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')  %1号车
fill([x2,x2,x2-L,x2-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'y')  %2号车

% 画分界线
plot([-5, len_line],[0, 0], 'w--', 'linewidth',2);  %分界线
plot([-5,len_line],[d,d],'w','linewidth',2);  %左边界线
plot([-5,len_line],[-d,-d],'w','linewidth',2);  %左边界线

% 设置坐标轴显示范围
axis equal % 使横纵坐标比例相等
set(gca, 'XLim',[-5 len_line]); 
set(gca, 'YLim',[-4 4]); 

%% 五次多项式轨迹生成

% 计算A和B两个系数矩阵
X = [state_t0(:,1); state_t1(:,1)];
Y = [state_t0(:,2); state_t1(:,2)];
T = [ t0^5      t0^4      t0^3     t0^2    t0   1;
      5*t0^4    4*t0^3    3*t0^2   2*t0    1    0;
      20*t0^3   12*t0^2   6*t0     1       0    0;
      t1^5      t1^4      t1^3     t1^2    t1   1;
      5*t1^4    4*t1^3    3*t1^2   2*t1    1    0;
      20*t1^3   12*t1^2   6*t1     1       0    0];
A = T \ X;
B = T \ Y;

% 将时间从t0到t1离散化，获得离散时刻的轨迹坐标
t=(t0:0.05:t1)';
path=zeros(length(t),4);%1-4列分别存放x,y,vx,vy 
for i = 1:length(t)
    % 纵向位置坐标
    path(i,1) = [t(i)^5, t(i)^4, t(i)^3, t(i)^2, t(i), 1] * A;
    
    % 横向位置坐标
    path(i,2) = [t(i)^5, t(i)^4, t(i)^3, t(i)^2, t(i), 1] * B;
    
    % 纵向速度
    path(i,3) = [5*t(i)^4,  4*t(i)^3,  3*t(i)^2,  2*t(i), 1, 0] * A;
    
    % 横向速度
    path(i,4) = [5*t(i)^4,  4*t(i)^3,  3*t(i)^2,  2*t(i), 1, 0] * B;
end

% 画换道轨迹 第一幅图
plot(path(:,1),path(:,2),'r--','linewidth',1.5); 

%% 分析速度

% 横向速度
figure 
plot(t, path(:,4), 'k'); 
xlabel('时间 / s ');
ylabel('横向速度 / m/s ');

% 纵向速度
figure 
plot(t, path(:,3), 'k'); 
xlabel('时间 / s ');
ylabel('纵向速度 / m/s ');

% matlab大量数据plot得到的是拟合曲线吗？不是的，这只不过是数据的散点图或连线图。要想得到拟合曲线，必须按下列步骤来求得：
% 第一步，将数据分别赋值给【x,y】
% 第二步，在不知数学模型的情况下，可以先用plot（x,y，'*'）命令，绘制散点图，分析并确定其走向（某数学函数的类型）
% 第三步，自定义该数学函数，即 func=@(p,x) 具体的数学函数表达式
% 第四步，用最小二乘求解非线性曲线拟合函数（lsqcurvefit），求解拟合系数，即
% p = lsqcurvefit(func,p0,xdata,ydata,lb,ub)
% 这里，p是拟合系数，func是自定义函数，p0是拟合系数的初值，xdata是x数据，ydata是y数据，lb是p值的上限值，ub是p值的下限值
% 第五步，得到p拟合系数，即可计算其拟合值（y1）,即 
% y1=func(p,x1) %这里x1可以是原数据的x值，也可以是x的最小值到最大值，等差值为d的向量值
% 第六步，此时用plot（x1,y1，'-'）命令，绘制的图形即为拟合曲线

% 与数据插值类似，曲线拟合也是一种函数逼近的方法。
% 最小二乘法（又称最小平方法）是一种数学优化技术。它通过最小优化误差的平方和来寻找数据的最佳函数匹配。
% polyfit():多项式拟合系数
% 功能：求得最小二乘拟合多项式系数
% x=[0.6,1.0,1.4,1.8,2.2,2.6,3.0,3.4,3.8,4];
% y=[0.08,0.22,0.31,0.4,0.48,0.56,0.67,0.75,0.8,1.0];
% p=polyfit(x,y,1);
% plot(x,y,'*',x,polyval(p,x));

