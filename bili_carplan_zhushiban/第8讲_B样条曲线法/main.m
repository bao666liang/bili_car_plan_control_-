% B样条曲线法
% 作者：Ally
% 日期：2021/2/6
clc
clear
close all

%% 数据定义
% 次数就是指样条函数在指定区间上是几次多项式，阶数就是指样条函数有几阶连续导数
% 如果把函数曲线看作是运动轨迹，那么切线方向就是速度方向。
% 导数连续，意味着运动方向的变化是连续的，没有尖锐转弯。
k = 4;                                    % 每个k阶B样条基函数是关于u的k-1次曲线 (不同于贝塞尔k阶k次)
flag = 1;                                  %1,2分别绘制均匀B样条曲线、准均匀B样条曲线(常用)
d = 3.5;
P=[0, 10, 25, 25, 40, 50;
    -d/2,-d/2,-d/2+0.5,d/2-0.5,d/2,d/2 ];   %n=5, 有6个控制点，可以满足曲率连续(位置)
n = size(P,2)-1;                          % n是控制点个数(下标)，从0开始计数

%% 生成B样条曲线

path=[];% 路径
Bik = zeros(n+1, 1);%B样条基函数

if flag == 1     % 均匀B样条
    NodeVector = linspace(0, 1, n+k+1); %节点矢量（均匀的）
    for u = (k-1)/(n+k+1) : 0.001 : (n+1)/(n+k+1) % open_B样条：即取控制点的有效区间交集为(u_k-1,u_n+1)
        for i = 0 : 1 : n
            Bik(i+1, 1) = BaseFunction(i, k-1 , u, NodeVector);% 计算出n+1个Bik
        end
        p_u = P * Bik;
        path = [path; [p_u(1,1),p_u(2,1)]];
    end
    
elseif flag == 2  % 准均匀B样条
    NodeVector = U_quasi_uniform(n, k-1); % 准均匀B样条的节点矢量(两端有重复度)
    for u = 0 : 0.005 : 1-0.005
        for i = 0 : 1 : n
            Bik(i+1, 1) = BaseFunction(i, k-1 , u, NodeVector);
        end
        p_u = P * Bik; % 位置坐标
        path=[path; [p_u(1),p_u(2)]];
    end
else
    fprintf('error!\n');
end

%% 画图
d = 3.5;               % 道路标准宽度
W = 1.8;               % 汽车宽度
L = 4.7;               % 车长
figure
len_line = 50;
P0 = [0, -d/2];

% 画灰色路面图
GreyZone = [-5,-d-0.5; -5,d+0.5; len_line,d+0.5; len_line,-d-0.5];
fill(GreyZone(:,1),GreyZone(:,2),[0.5 0.5 0.5]);
hold on
fill([P0(1),P0(1),P0(1)-L,P0(1)-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')  

% 画分界线
plot([-5, len_line],[0, 0], 'w--', 'linewidth',2);  %分界线
plot([-5,len_line],[d,d],'w','linewidth',2);     %左边界线
plot([-5,len_line],[-d,-d],'w','linewidth',2);  %左边界线

% 设置坐标轴显示范围
axis equal
set(gca, 'XLim',[-5 len_line]); 
set(gca, 'YLim',[-4 4]); 

% 绘制路径
scatter(path(:,1),path(:,2),100, '.b');%路径点
scatter(P(1,:),P(2,:),'g')
plot(P(1,:),P(2,:),'r');%路径点


