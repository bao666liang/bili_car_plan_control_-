% 纯跟踪（Pure Pursuit）法
% 作者：Ally
% 日期：20210429
clc
clear
close all
load  path.mat

%% 相关参数定义
RefPos = path;         % 导入的一系列路径点
targetSpeed = 10;      % m/s
Kv = 0.1;              % 前视距离系数
Kp = 0.8;              % 速度P控制器系数
Ld0 = 2;               % Ld0是预瞄距离的下限值
dt = 0.1;              % 时间间隔，单位：s
L = 2.9;               % 车辆轴距，单位：m

% 计算参考航向角
diff_x = diff(RefPos(:,1)) ;% 计算相邻点的差值
diff_x(end+1) = diff_x(end);
diff_y = diff(RefPos(:,2)) ;
diff_y(end+1) = diff_y(end);
% 因为路径点距离足够近就可以将正切方向看成该点的切线，即期望移动方向，就是该点期望航向角
refHeading = atan2(diff_y , diff_x); % 参考航向角（相邻路径点的正切角）用于计算车和前视点的横向距离

%% 主程序 只考虑运动学约束

% 车辆初始状态定义
pos = RefPos(1,:)+1;%可以在参考路径上也可以不在
v = 0;
heading = 0.02;% 在world下的车辆偏航角(朝向)
 
% 将初始状态纳入实际状态数组中
pos_actual = pos;
heading_actual = heading;
v_actual = v;
idx = 1;
latError_PP = [];

% 循环遍历轨迹点
while idx < size(RefPos,1)-1
    % 寻找预瞄距离范围内最近路径点
    [lookaheadPoint,idx] = findLookaheadPoint(pos, v, RefPos,Kv, Ld0);
   
    % 计算控制量
    [delta,latError]  = pure_pursuit_control(lookaheadPoint,idx,pos, heading, v, RefPos,refHeading, Kv, Ld0,L);
    % detla = (2*L/Ld^2)*latError 纯跟踪本质上是P控制器 见129行
    % 如果误差过大，退出循迹
    if abs(latError) > 3
        disp('误差过大，退出程序!\n')
        break
    end
    
    % 计算加速度 到达期望速度后加速度为0（即跟踪期望纵向速度）
    % a是控制量=K*err_v,比例控制器控制纵向速度
    a = Kp* (targetSpeed-v)/dt;
    
    % 更新状态量
    [pos, heading, v] = updateState(a,pos, heading, v,delta,L, dt);
    
    % 保存每一步的实际量
    pos_actual(end+1,:) = pos;
    heading_actual(end+1,:) = heading;
    v_actual(end+1,:) = v;
    latError_PP(end+1,:) = [idx,latError];
end

% 画图
figure
plot(RefPos(:,1), RefPos(:,2), 'b');
xlabel('纵向坐标 / m');
ylabel('横向坐标 / m');
hold on 
for i = 1:size(pos_actual,1)
    scatter(pos_actual(i,1), pos_actual(i,2),150, '.r');
    pause(0.05)
end
legend('规划车辆轨迹', '实际行驶轨迹')

% 保存
path_PP = pos_actual;
save path_PP.mat path_PP
save latError_PP.mat latError_PP

%% 首先在参考轨迹上搜索离当前车辆位置最近的点 （因为用前视距离画圆弧不一定经过path点）
function  [lookaheadPoint,idx_target] = findLookaheadPoint(pos, v, RefPos, Kv, Ld0)

% 找到距离当前位置最近的一个参考轨迹点的序号（相当于车的位置就是最近参考轨迹点？？？）
sizeOfRefPos = size(RefPos,1);
for i = 1:sizeOfRefPos
    dist(i,1) = norm(RefPos(i,:) - pos);   
end
[~,idx] = min(dist); 


% 从该点开始向轨迹前方搜索，找到与预瞄距离最相近的一个轨迹点
L_steps = 0;           % 参考轨迹上几个相邻点的累计距离
Ld = Kv*v + Ld0;       % Ld0是预瞄距离的下限值；
while L_steps < Ld && idx < sizeOfRefPos
    L_steps = L_steps + norm(RefPos(idx + 1,:) - RefPos(idx,:));
    idx = idx+1;
end
idx_target = idx;
lookaheadPoint = RefPos(idx,:);
end


%% 获得控制量：前轮转向
function [delta,latError] = pure_pursuit_control(lookaheadPoint,idx,pos, heading, v,RefPos,refHeading, Kv, Ld0, L)
sizeOfRefPos = size(RefPos,1);
if idx < sizeOfRefPos % idx最大就是size0fRefPos
    Point_temp = lookaheadPoint;
else
    Point_temp = RefPos(end,1:2);%车辆离末点小于Ld时就一直视最后一个点为前视点
end
% heading规定从x轴逆时针为正，OR一定垂直于车轴L
alpha = atan2(Point_temp(1,2) - pos(2), Point_temp(1,1) - pos(1))  - heading;
Ld = Kv*v + Ld0;

% 求位置、航向角的误差
x_error  = pos(1) - RefPos(idx,1);
y_error = pos(2) - RefPos(idx,2);
heading_r = refHeading(idx);

% 根据百度Apollo，计算横向误差:前视点垂直于L的距离
% 见 https://zhuanlan.zhihu.com/p/538669721 ,P向n_m方向投影
latError = y_error*cos(heading_r) - x_error*sin(heading_r);
%latError = Ld*sin(alpha); 这个对了？？？？
% 前轮转角 用公式5更新的控制量detla ！！！！！
delta = atan2(2*L*sin(alpha), Ld);
%delta = (2*L/Ld^2)*latError; %纯跟踪本质上是P控制器  但是latError要用式6
end

%% 更新状态量
function [pos_new, heading_new, v_new] = updateState(a,pos_old, heading_old, v_old,delta,wheelbase, dt)
pos_new(1) = pos_old(1) + v_old*cos(heading_old)*dt;
pos_new(2) =  pos_old(2) + v_old*sin(heading_old)*dt;
% 前轮朝向线速度等于后轮朝向线速度，因此v/R=w ，当detla=0后heading不变
heading_new=  heading_old + v_old*dt*tan(delta)/wheelbase;
v_new =  v_old + a*dt;
end

