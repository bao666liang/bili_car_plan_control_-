% A*�㷨
% ���ߣ�Ally
% ���ڣ�2021/1/9
clc
clear
close all

%% ����ͼ

% դ���ͼ����������������
m = 5;
n = 7;
start_node = [2, 3];% ��ʾ���Ǹ�����λ�ã�����plot����
target_node = [6, 3];
obs = [4,2; 4,3; 4,4];
% ���Ƹ��ӣ�����
for i = 1:m
    plot([0,n], [i, i], 'k');% ���������� ��x = 0,y = 1����x = 7,y = 1��Ȼ����������
    hold on
end
    
for j = 1:n
     plot([j, j], [0, m], 'k');
end

axis equal % ʹ�������굥λһ�� ����ÿ�����Ӷ�Ϊ������
xlim([0, n]);% ������������
ylim([0, m]);   

% �����ϰ����ֹ����ɫ�� ��fill(X,Y,c):�ø���plot���������������
fill([start_node(1)-1, start_node(1), start_node(1), start_node(1)-1],...
    [start_node(2)-1, start_node(2)-1 , start_node(2), start_node(2)], 'g');

fill([target_node(1)-1, target_node(1), target_node(1), target_node(1)-1],...
    [target_node(2)-1, target_node(2)-1 , target_node(2), target_node(2)], 'r');

for i = 1:size(obs,1)
    temp = obs(i,:);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'b');
end

%% Ԥ����

% ��ʼ��closeList��������߹��ڵ�
closeList = start_node;
closeList_path = {start_node,start_node};% Ԫ������һ��Ϊ�ڵ㣬�ڶ���Ϊ��㵽�ýڵ������·��
closeList_cost = 0;% ����
child_nodes = child_nodes_cal(start_node,  m, n, obs, closeList); % ���������ڽڵ�

% ��ʼ��openList��������п����ڽڵ�
openList = child_nodes;
for i = 1:size(openList,1)
    openList_path{i,1} = openList(i,:);
    openList_path{i,2} = [start_node;openList(i,:)];% �ڶ���Ϊ��㵽���ӽڵ��·��
end

for i = 1:size(openList, 1) 
    g = norm(start_node - openList(i,1:2));% ŷ�Ͼ��루��㵽�ýڵ㣩
    h = abs(target_node(1) - openList(i,1)) + abs(target_node(2) - openList(i,2));%�����پ���
    f = g + h;
    openList_cost(i,:) = [g, h, f];% ���ÿ���ӽڵ��g/h/f
end

%% ��ʼ����
% ��openList��ʼ�����ƶ�������С�Ľڵ㣬Ȼ�󽫸���С�ڵ���Ϊ��һ�����ڵ�
[~, min_idx] = min(openList_cost(:,3));
parent_node = openList(min_idx,:);


%% ����ѭ��
flag = 1;
while flag   
    
    % �ҳ��ø��ڵ���չ���ӽڵ㣨�Ѿ�ɾ������closeList���ظ��ĺ��ϰ���ڵ㣩
    child_nodes = child_nodes_cal(parent_node,  m, n, obs, closeList); 
    
    % �ж���Щ�ӽڵ��Ƿ���openList�У����ڣ���Ƚϸ��£�û����׷�ӵ�openList�У���������
    for i = 1:size(child_nodes,1)
        child_node = child_nodes(i,:);
        [in_flag,openList_idx] = ismember(child_node, openList, 'rows');
        g = openList_cost(min_idx, 1) + norm(parent_node - child_node);%������ӽڵ��g
        h = abs(child_node(1) - target_node(1)) + abs(child_node(2) - target_node(2));
        f = g+h;
         % ���ڣ�����openList_cost�е�g��f
         % (����㵽���ڵ�Ĵ��ۼ��ϸ��ڵ㵽���ӽڵ�Ĵ���
         %  С����㵽���ӽڵ�Ĵ���(��Ϊ���ӽڵ�֮ǰ��openlist���Ѿ����g��)�͸���g��ͬʱ����f,path)
        if in_flag          
            if g < openList_cost(openList_idx,1)
                openList_cost(openList_idx, 1) = g;
                openList_cost(openList_idx, 3) = f;
                % ���ǵ�֮ǰ����㵽��child_node��·�����ڸ�child_nodeǰ�����㵽���ڵ�·��������������
                openList_path{openList_idx,2} = [openList_path{min_idx,2}; child_node];
            end
        else    % �����ڣ������ӽڵ�׷�ӵ�openListβ�У�ͬʱ������g/h/f
            openList(end+1,:) = child_node;
            openList_cost(end+1, :) = [g, h, f];
            openList_path{end+1, 1} = child_node;
            openList_path{end, 2} = [openList_path{min_idx,2}; child_node];
        end
    end
   
   
    % ��openList�Ƴ���һ���ӽڵ�ĸ��ڵ㵽 closeList����Ϊmin_idx����һ��ʼ��parent_node//67�У�����
    closeList(end+1,: ) =  openList(min_idx,:);
    closeList_cost(end+1,1) =   openList_cost(min_idx,3);
    closeList_path(end+1,:) = openList_path(min_idx,:);
    openList(min_idx,:) = [];
    openList_cost(min_idx,:) = [];
    openList_path(min_idx,:) = [];
 
    % ������������openList�����ƶ�������С�Ľڵ���Ϊ��һ�����ڵ�
    [~, min_idx] = min(openList_cost(:,3));
    parent_node = openList(min_idx,:);
    
    % �ж��Ƿ��������յ�
    if parent_node == target_node
        closeList(end+1,: ) =  openList(min_idx,:);
        closeList_cost(end+1,1) =   openList_cost(min_idx,1);
        closeList_path(end+1,:) = openList_path(min_idx,:);
        flag = 0; % �˳�ѭ��
    end
end
    

%% ��·��
path_opt = closeList_path{end,2};
path_opt(:,1) = path_opt(:,1)-0.5;% ������Ȼ��������(2,3)������plot�л��е�Ϊ(1.5,2.5)
path_opt(:,2) = path_opt(:,2)-0.5;
scatter(path_opt(:,1), path_opt(:,2), 'k');% ���������ɢ��
plot(path_opt(:,1), path_opt(:,2), 'k'); % ����ɢ��
  