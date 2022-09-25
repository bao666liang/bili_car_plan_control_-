% DP算法
% 作者：Ally
% 日期：2021/1/1
clc
clear
close all

%% 阶段-状态定义
stages = 5;% 通常不会知道阶段下的状态，要根据约束计算
nodes_dist = cell(stages,3);

% 第1阶段
nodes_dist{1,1} = 1;% 该阶段的状态 A
nodes_dist{1,2} = [1,2,3];% 下一阶段的状态（B1,B2,B3）
nodes_dist{1,3} = [2,5,1];% 路径代价

% 第2阶段
nodes_dist{2,1} = [1;2;3];
nodes_dist{2,2} = [1,2,3];
nodes_dist{2,3} = [12, 14, 10; 6, 10, 4; 13, 12, 11];% 案例为全连接

% 第3阶段
nodes_dist{3,1} = [1;2;3];
nodes_dist{3,2} = [1,2];
nodes_dist{3,3} = [3, 9; 6, 5; 8, 10];

% 第4阶段
nodes_dist{4,1} = [1;2];
nodes_dist{4,2} = 1;
nodes_dist{4,3} = [5; 2];

% 第5阶段
nodes_dist{5,1} = 1;
nodes_dist{5,2} = 1;
nodes_dist{5,3} = 0;

% 最优路径及其距离值定义
path = cell(stages, 2);
dist = cell(stages, 2);
for i = 1:stages-1
    dist{i, 1} = nodes_dist{i,1};% 第一列：该阶段的状态
    dist{i, 2} = inf(length(dist{i, 1}), 1);% 第二列：该阶段的每个状态到最终阶段的最优距离值 初始化为inf
    path{i, 1} = nodes_dist{i,1};% 第一列：该阶段的状态
end
% 根据最后一个阶段(因为下一个阶段还是本状态 1)，直接初始化
dist{stages, 1} = 1;  
dist{stages, 2} = 0;  
path{stages, 1} = 1;
path{stages, 2} = 1;% 第二列 ：该阶段的每个状态到最终阶段的状态的最优路径(根据dist{i,2})


%% 逆向寻优

% 第一层循环：逆向遍历每一个阶段
for i = stages-1:-1:1 % 逆向 因为第五个阶段已经初始化，从阶段四开始
    num_states_f = length(nodes_dist{i, 1});  % 每个阶段的状态(数)

    % 第二层循环：遍历第i阶段的每一个状态
    for j = 1:num_states_f
        num_states_r = length(nodes_dist{i+1, 1}); % 第i+1(下)阶段的状态数      
        
        % 第三层循环：遍历第i阶段的第j个状态到第i+1阶段的每(k)个状态的每一条路径
        for k = 1:num_states_r
            % 如:min{{d(B1,C1)+f(C1)},{d(B1,C2)+f(C2)},{d(B1,C3)+f(C2)}}
            if  nodes_dist{i,3}(j,k) + dist{i+1,2}(k,1) < dist{i,2}(j,1)% 因为一开始为inf
                dist{i,2}(j,1) = nodes_dist{i,3}(j,k) + dist{i+1,2}(k,1);
                path{i, 2}(j,:) = [j, path{i+1, 2}(k,:)];% 将该状态添加到本阶段最优路径开头
            end
        end
    end
end
            
%% 正向求解
path_opt =  path(1,:);    
dist_opt =  dist{1,2};            
  