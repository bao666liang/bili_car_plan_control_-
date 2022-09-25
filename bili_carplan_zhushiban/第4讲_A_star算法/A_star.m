% A*算法
% 作者：Ally
% 日期：2021/1/9
clc
clear
close all

%% 画地图

% 栅格地图的行数、列数定义
m = 5;
n = 7;
start_node = [2, 3];% 表示的是格子数位置，不是plot坐标
target_node = [6, 3];
obs = [4,2; 4,3; 4,4];
% 绘制格子，画线
for i = 1:m
    plot([0,n], [i, i], 'k');% 画五条横线 （x = 0,y = 1）（x = 7,y = 1）然后将两点连接
    hold on
end
    
for j = 1:n
     plot([j, j], [0, m], 'k');
end

axis equal % 使横纵坐标单位一致 即让每个格子都为正方形
xlim([0, n]);% 限制坐标区间
ylim([0, m]);   

% 绘制障碍物、起止点颜色块 ，fill(X,Y,c):用格子plot顶点坐标填充多边形
fill([start_node(1)-1, start_node(1), start_node(1), start_node(1)-1],...
    [start_node(2)-1, start_node(2)-1 , start_node(2), start_node(2)], 'g');

fill([target_node(1)-1, target_node(1), target_node(1), target_node(1)-1],...
    [target_node(2)-1, target_node(2)-1 , target_node(2), target_node(2)], 'r');

for i = 1:size(obs,1)
    temp = obs(i,:);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'b');
end

%% 预处理

% 初始化closeList，存放已走过节点
closeList = start_node;
closeList_path = {start_node,start_node};% 元胞：第一列为节点，第二列为起点到该节点的最优路径
closeList_cost = 0;% 代价
child_nodes = child_nodes_cal(start_node,  m, n, obs, closeList); % 搜索可行邻节点

% 初始化openList，存放所有可行邻节点
openList = child_nodes;
for i = 1:size(openList,1)
    openList_path{i,1} = openList(i,:);
    openList_path{i,2} = [start_node;openList(i,:)];% 第二列为起点到该子节点的路径
end

for i = 1:size(openList, 1) 
    g = norm(start_node - openList(i,1:2));% 欧氏距离（起点到该节点）
    h = abs(target_node(1) - openList(i,1)) + abs(target_node(2) - openList(i,2));%曼哈顿距离
    f = g + h;
    openList_cost(i,:) = [g, h, f];% 存放每个子节点的g/h/f
end

%% 开始搜索
% 从openList开始搜索移动代价最小的节点，然后将该最小节点作为下一个父节点
[~, min_idx] = min(openList_cost(:,3));
parent_node = openList(min_idx,:);


%% 进入循环
flag = 1;
while flag   
    
    % 找出该父节点扩展的子节点（已经删除了与closeList中重复的和障碍物节点）
    child_nodes = child_nodes_cal(parent_node,  m, n, obs, closeList); 
    
    % 判断这些子节点是否在openList中，若在，则比较更新；没在则追加到openList中！！！！！
    for i = 1:size(child_nodes,1)
        child_node = child_nodes(i,:);
        [in_flag,openList_idx] = ismember(child_node, openList, 'rows');
        g = openList_cost(min_idx, 1) + norm(parent_node - child_node);%计算该子节点的g
        h = abs(child_node(1) - target_node(1)) + abs(child_node(2) - target_node(2));
        f = g+h;
         % 若在，更新openList_cost中的g和f
         % (即起点到父节点的代价加上父节点到该子节点的代价
         %  小于起点到该子节点的代价(因为该子节点之前在openlist中已经算过g了)就更新g，同时更新f,path)
        if in_flag          
            if g < openList_cost(openList_idx,1)
                openList_cost(openList_idx, 1) = g;
                openList_cost(openList_idx, 3) = f;
                % 覆盖掉之前的起点到该child_node的路径，在该child_node前添加起点到父节点路径！！！！！！
                openList_path{openList_idx,2} = [openList_path{min_idx,2}; child_node];
            end
        else    % 若不在，将该子节点追加到openList尾行，同时计算其g/h/f
            openList(end+1,:) = child_node;
            openList_cost(end+1, :) = [g, h, f];
            openList_path{end+1, 1} = child_node;
            openList_path{end, 2} = [openList_path{min_idx,2}; child_node];
        end
    end
   
   
    % 从openList移除这一轮子节点的父节点到 closeList，因为min_idx就是一开始的parent_node//67行！！！
    closeList(end+1,: ) =  openList(min_idx,:);
    closeList_cost(end+1,1) =   openList_cost(min_idx,3);
    closeList_path(end+1,:) = openList_path(min_idx,:);
    openList(min_idx,:) = [];
    openList_cost(min_idx,:) = [];
    openList_path(min_idx,:) = [];
 
    % 重新搜索：从openList搜索移动代价最小的节点作为下一个父节点
    [~, min_idx] = min(openList_cost(:,3));
    parent_node = openList(min_idx,:);
    
    % 判断是否搜索到终点
    if parent_node == target_node
        closeList(end+1,: ) =  openList(min_idx,:);
        closeList_cost(end+1,1) =   openList_cost(min_idx,1);
        closeList_path(end+1,:) = openList_path(min_idx,:);
        flag = 0; % 退出循环
    end
end
    

%% 画路径
path_opt = closeList_path{end,2};
path_opt(:,1) = path_opt(:,1)-0.5;% 比如虽然起点格子是(2,3)，但在plot中画中点为(1.5,2.5)
path_opt(:,2) = path_opt(:,2)-0.5;
scatter(path_opt(:,1), path_opt(:,2), 'k');% 画出上面的散点
plot(path_opt(:,1), path_opt(:,2), 'k'); % 连接散点
  