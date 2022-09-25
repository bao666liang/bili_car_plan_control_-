function    [Delta_real,v_real,idx,latError,U ] = ...
    mpc_control(x,y,yaw,refPos_x,refPos_y,refPos_yaw,refDelta,dt,L,U,target_v)
%% MPC预设参数
Nx = 3;         % 状态量的个数
Nu = 2;         % 控制量的个数
Np = 60;        % 预测步长
Nc = 30;        % 控制步长
row = 10;       % 松弛因子
% 克罗内克积:Qq=I_Np⊙Q,其中I为Np*Np,Q为Nx*Nx;Rr同理
Q = 100*eye(Np*Nx);      % (Np*Nx) × (Np*Nx) 更改Q/R改变跟随效果
R = 1*eye(Nc*Nu);        % (Nc*Nu) × (Nc*Nu)

% 控制量约束条件
umin = [-0.2; -0.54];
umax = [0.2; 0.332];
delta_umin = [-0.05; -0.64];
delta_umax = [0.05; 0.64];

%% 原运动学误差状态空间方程的相关矩阵
% 计算参考控制量
idx = calc_target_index(x,y,refPos_x,refPos_y); % 最近参考点索引值
v_r = target_v;
Delta_r = refDelta(idx);% 船舶可以用LOS求
heading_r = refPos_yaw(idx);

% 实际状态量与参考状态量
X_real = [x,y,yaw];
Xr = [refPos_x(idx), refPos_y(idx), refPos_yaw(idx)];

% 求位置、航向角的误差
x_error  = x - refPos_x(idx);
y_error = y - refPos_y(idx);

% 根据百度Apolo，计算横向误差
latError = y_error*cos(heading_r) - x_error*sin(heading_r);

% a,b两个矩阵
a = [1    0   -v_r*sin(heading_r)*dt;
     0    1   v_r*cos(heading_r)*dt;
     0    0   1];
b = [cos(heading_r)*dt     0;
     sin(heading_r)*dt     0;
     tan(heading_r)*dt/L   v_r*dt/(L * (cos(Delta_r)^2))];

%% 新的状态空间方程的相关矩阵
% 新的状态量
kesi = zeros(Nx+Nu,1);              % (Nx+Nu) × 1
kesi(1:Nx) = X_real - Xr;
kesi(Nx+1:end) = U;

% 新的A矩阵
A_cell = cell(2,2);
A_cell{1,1} = a;
A_cell{1,2} = b;
A_cell{2,1} = zeros(Nu,Nx);
A_cell{2,2} = eye(Nu);
A = cell2mat(A_cell);    % (Nx+Nu) × (Nx+Nu) 将元胞数组转为矩阵

% 新的B矩阵
B_cell = cell(2,1);
B_cell{1,1} = b;
B_cell{2,1} = eye(Nu);
B = cell2mat(B_cell);           % (Nx+Nu) × Nu

% 新的C矩阵
C = [eye(Nx), zeros(Nx, Nu)];   % Nx × (Nx+Nu)

% PHI矩阵
PHI_cell = cell(Np,1);
for i = 1:Np
    PHI_cell{i,1}=C*A^i;  % Nx × (Nx+Nu)
end
PHI = cell2mat(PHI_cell);   % (Nx * Np) × (Nx + Nu)


% THETA矩阵
THETA_cell = cell(Np,Nc);
for i = 1:Np
    for j = 1:Nc
        if j <= i
            THETA_cell{i,j} = C*A^(i-j)*B;    % Nx × Nu
        else
            THETA_cell{i,j} = zeros(Nx,Nu);
        end
    end
end
THETA = cell2mat(THETA_cell);                 % (Nx * Np) × (Nu * Nc)


%% 二次型目标函数的相关矩阵

% H矩阵
H_cell = cell(2,2); % 定义为2*2是为了加入松弛因子row，以此用quadprog()求二次规划时有解
H_cell{1,1} = THETA'*Q*THETA + R;  % (Nu * Nc) × (Nu * Nc)
H_cell{1,2} = zeros(Nu*Nc,1);
H_cell{2,1} = zeros(1,Nu*Nc);
H_cell{2,2} = row;% 和Qq构成对角(正定)
H = cell2mat(H_cell);            % (Nu * Nc + 1) × (Nu * Nc + 1)

% E矩阵
E = PHI*kesi;                    % (Nx * Np) × 1

% g矩阵 因为H变了，g矩阵同样要变化
g_cell = cell(1,2);
g_cell{1,1} = E'*Q*THETA;          % 1×(Nu * Nc )，g'行数为了和H的列数匹配，新添加一列0，g'是添加一行
g_cell{1,2} = 0;
g = cell2mat(g_cell);              % 1×(Nu * Nc + 1 ) 

%% 约束条件的相关矩阵

% A_I矩阵
A_t = zeros(Nc,Nc);     % 下三角单位方阵
for i = 1:Nc
    A_t(i,1:i) = 1;
end
A_I = kron(A_t,eye(Nu));       % (Nu * Nc) × (Nu * Nc) 克罗内克积求A_I

% Ut矩阵 由上一时刻控制量u(k-1)组成
Ut = kron(ones(Nc,1),U);       % (Nu * Nc) × 1

% 控制量与控制量变化量的约束
Umin = kron(ones(Nc,1),umin);
Umax = kron(ones(Nc,1),umax);
delta_Umin = kron(ones(Nc,1),delta_umin);
delta_Umax = kron(ones(Nc,1),delta_umax);

% 用于quadprog函数不等式约束Ax <= b的矩阵A
A_cons_cell = {A_I, zeros(Nu*Nc,1);       % 列数为了和H的列数匹配，新添加一列0，(Nu * Nc) × (Nu * Nc), (Nu * Nc) ×1
    -A_I, zeros(Nu*Nc,1)}; 
A_cons = cell2mat(A_cons_cell);           % (Nu * Nc * 2) × (Nu * Nc +1)

% 用于quadprog函数不等式约束Ax <= b的向量b
b_cons_cell = {Umax-Ut;
    -Umin+Ut};
b_cons = cell2mat(b_cons_cell);

% △U的上下界约束
lb = delta_Umin;
ub = delta_Umax;

%% 开始求解过程

options = optimoptions('quadprog','Display','iter','MaxIterations',100,'TolFun',1e-16);
delta_U = quadprog(H,g,A_cons,b_cons,[],[],lb,ub,[],options);   %(Nu * Nc +1) × 1 加一是因为row

%% 计算输出

% 根据MPC只选取求解的delta_U的第一组控制量。注意：这里是v_tilde的变化量和Delta_tilde的变化量
delta_v_tilde = delta_U(1); % 速度误差量的变化量
delta_Delta_tilde = delta_U(2);% 前轮转角误差量的变化量

% 更新这一时刻的控制量。注意，这里的“控制量”是v_tilde和Delta_tilde，而不是真正的v和Delta
U(1) = kesi(4) + delta_v_tilde;  % kesi向量的4和5行就是上一时刻的tilde_u
U(2) = kesi(5) + delta_Delta_tilde;  

% 求解真正的控制量v_real和Delta_real进行控制输出
v_real = U(1) + v_r; 
Delta_real = U(2) + Delta_r;

end