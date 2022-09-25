% ACA 算法 （类似遗传，粒子群等智能算法都是相同思路，更新方式不同，次优，Djstla为22）
% 作者：Ally
% 日期：2020/12/26

clc
clear
close all


%% 初始化参数
% 根据节点的邻近节点表及字母节点-数字节点对应表，构造节点元胞数组
nodes_data = cell(0);
nodes_data(1,:) = {1, [2, 6, 7], [12, 16, 14]};
nodes_data(2,:) = {2, [1, 3, 6], [12, 10, 7]};
nodes_data(3,:) = {3, [2, 4, 5, 6], [10, 3, 5, 6]};
nodes_data(4,:) = {4, [3, 5], [3, 4]};
nodes_data(5,:) = {5, [3, 4, 6, 7], [5, 4, 2, 8]};
nodes_data(6,:) = {6, [1, 2, 3, 5, 7], [16, 7, 6, 2, 9]};
nodes_data(7,:) = {7, [1, 5, 6], [14, 8, 9]};

% 始末节点
node_start = 4;                       % 初始源节点
node_end = 1;                         % 终节点

% 蚁群相关定义
m = 50;                              % 蚂蚁数量
n = size(nodes_data,1);              % 节点数量7
alpha = 1;                           % 信息素重要程度因子
beta = 5;                            % 启发函数重要程度因子
rho = 0.1;                           % 信息素挥发因子
Q = 1;                               % 常数

% 迭代过程中，相关参数初始化定义
iter = 1;                            % 迭代次数初值
iter_max = 100;                      % 最大迭代次数 
Route_best = cell(iter_max,1);       % 各代最佳路径 (50只都走完一轮为一代)     
Length_best = zeros(iter_max,1);     % 各代最佳路径的长度  
Length_ave = zeros(iter_max,1);      % 各代路径的平均长度

% 将信息素、挥发因子一并放入nodes_data中，方便访问遍历（91行，用于计算下一节点访问概率）
Delta_Tau_initial = nodes_data(:,1:2);%第一列和第二列
for i = 1:size(nodes_data,1)
    nodes_data{i,4} = ones(1, length(nodes_data{i,3}));   % 信息素初始化（第四列）
    nodes_data{i,5} = 1./nodes_data{i,3};                 % 挥发因子（第五列）
    Delta_Tau_initial{i,3} = zeros(1, length(nodes_data{i,3}));%第三列为所有邻节点路径信息素浓度初始化
end


%% 迭代寻找最佳路径
while iter <= iter_max  %第一层大循环
  
    route = cell(0);%因为每个蚂蚁访问路径长度(节点数)不同，因此用元胞矩阵存储不同蚂蚁访问路径
    
    %%  逐个蚂蚁路径选择 第二层循环
    for i = 1:m
        % 逐个节点路径选择
        neighbor = cell(0);%邻节点
        node_step = node_start;%源节点
        path = node_step;%走过的路径节点集合，后面有更新path(end+1) = node_step，但换一只蚂蚁就要从头走
 
        dist = 0;
        % ismember(A,B) 确定A(1*4)的哪些元素也在B中，如返回(1 0 1 0),1位置为共有元素
        %%第三层循环 单只蚂蚁搜寻路径
        while ~ismember(node_end, path) %当路径表里面包含了终节点时，该蚂蚁完成路径寻优，跳出循环
           
            % 寻找邻近节点           
            neighbor = nodes_data{node_step,2};
            
            % 同时删除neighbor中已经访问过的邻近节点
            idx = [];
            for k = 1:length(neighbor)
                if ismember(neighbor(k), path)
                    idx(end+1) =  k;
                end
            end
            neighbor(idx) = [];
            
            % 判断是否进入死胡同(即邻近节点都是已走过的节点)， 若是，直接返回到起点，重新寻路
            if isempty(neighbor)
                neighbor = cell(0);
                node_step = node_start;
                path = node_step;
                dist = 0;
                continue
            end
                
            
            %计算下一个节点的访问概率
            P = neighbor; %第一行为所有待访问邻节点
            for k=1:length(P) 
                P(2,k) = nodes_data{node_step, 4}(k)^alpha * ...
                    nodes_data{node_step, 5}(k)^beta;
            end
            P(2,:) = P(2,:)/sum(P(2,:));%第二行为对应邻节点的访问概率
            
            % 轮盘赌法选择下一个访问节点（概率越大占得区间越多）
            Pc = cumsum(P(2,:));%求和：[0.1,0.3,0.6]->[0.1,0.4,1]
            Pc = [0, Pc];
            randnum = rand;%生成一个(0,1)间的随机数
            for k = 1:length(Pc)-1
                if randnum > Pc(k) && randnum < Pc(k+1)
                    target_node = neighbor(k);%随机数在哪个区间就选哪个邻节点
                end
            end
            
            
            % 计算走过的距离
            idx_temp = find(nodes_data{node_step, 2} == target_node);%find查找非零元素，返回索引值
            dist = dist + nodes_data{node_step, 3}(idx_temp);
            
            % 更新下一步的目标节点及路径集合            
            node_step = target_node;
            path(end+1) = node_step;         
                       
        end
        
        % 存放第i只蚂蚁的累计距离及对应路径（50*1）
        Length(i,1) = dist;
        route{i,1} = path;
    end
    
    %% 计算这一代的m只蚂蚁中最短距离及对应路径
    if iter == 1 % 第一代
        [min_Length,min_index] = min(Length);
        Length_best(iter) = min_Length;
        Length_ave(iter) = mean(Length);
        Route_best{iter,1} = route{min_index,1};
        
    else % 更新本次迭代最优：比较本次最优和上次最优的路径的取其中较短的来作为本次最优路径，直至收敛
        [min_Length,min_index] = min(Length);
        Length_best(iter) = min(Length_best(iter - 1),min_Length);
        Length_ave(iter) = mean(Length);
        if Length_best(iter) == min_Length
            Route_best{iter,1} = route{min_index,1};
        else
            Route_best{iter,1} = Route_best{iter-1,1};
        end
    end
    
    %% 更新信息素（本次迭代）
    
    % 计算每一条路径上的经过的蚂蚁留下的信息素
    Delta_Tau = Delta_Tau_initial;    
    
    % 逐个蚂蚁计算
    for i = 1:m
       
        % 逐个节点间的信息素
        for j = 1:length(route{i,1})-1
            node_start_temp = route{i,1}(j);
            node_end_temp = route{i,1}(j+1);
            idx =  find(Delta_Tau{node_start_temp, 2} == node_end_temp);
            % Q/Length(i)表示总路径越短其值越大，相对应两个节点间信息素越大，逐渐收敛到该路径
            Delta_Tau{node_start_temp,3}(idx) = Delta_Tau{node_start_temp,3}(idx) + Q/Length(i);
        end
        
    end
    
    % 考虑挥发因子，更新信息素
    for i = 1:size(nodes_data, 1)
        nodes_data{i, 4} = (1-rho) * nodes_data{i, 4} + Delta_Tau{i, 3};
    end
    
    % 更新迭代次数
    iter = iter + 1;
end


%% 绘图、结果    

figure
plot(1:iter_max,Length_best,'b',1:iter_max,Length_ave,'r')
legend('最短距离','平均距离')
xlabel('迭代次数')
ylabel('距离')
title('各代最短距离与平均距离对比')

% 最优路径
[dist_min, idx] = min(Length_best);
path_opt = Route_best{idx,1};
