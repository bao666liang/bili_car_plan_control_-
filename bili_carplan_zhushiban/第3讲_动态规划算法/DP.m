% DP�㷨
% ���ߣ�Ally
% ���ڣ�2021/1/1
clc
clear
close all

%% �׶�-״̬����
stages = 5;% ͨ������֪���׶��µ�״̬��Ҫ����Լ������
nodes_dist = cell(stages,3);

% ��1�׶�
nodes_dist{1,1} = 1;% �ý׶ε�״̬ A
nodes_dist{1,2} = [1,2,3];% ��һ�׶ε�״̬��B1,B2,B3��
nodes_dist{1,3} = [2,5,1];% ·������

% ��2�׶�
nodes_dist{2,1} = [1;2;3];
nodes_dist{2,2} = [1,2,3];
nodes_dist{2,3} = [12, 14, 10; 6, 10, 4; 13, 12, 11];% ����Ϊȫ����

% ��3�׶�
nodes_dist{3,1} = [1;2;3];
nodes_dist{3,2} = [1,2];
nodes_dist{3,3} = [3, 9; 6, 5; 8, 10];

% ��4�׶�
nodes_dist{4,1} = [1;2];
nodes_dist{4,2} = 1;
nodes_dist{4,3} = [5; 2];

% ��5�׶�
nodes_dist{5,1} = 1;
nodes_dist{5,2} = 1;
nodes_dist{5,3} = 0;

% ����·���������ֵ����
path = cell(stages, 2);
dist = cell(stages, 2);
for i = 1:stages-1
    dist{i, 1} = nodes_dist{i,1};% ��һ�У��ý׶ε�״̬
    dist{i, 2} = inf(length(dist{i, 1}), 1);% �ڶ��У��ý׶ε�ÿ��״̬�����ս׶ε����ž���ֵ ��ʼ��Ϊinf
    path{i, 1} = nodes_dist{i,1};% ��һ�У��ý׶ε�״̬
end
% �������һ���׶�(��Ϊ��һ���׶λ��Ǳ�״̬ 1)��ֱ�ӳ�ʼ��
dist{stages, 1} = 1;  
dist{stages, 2} = 0;  
path{stages, 1} = 1;
path{stages, 2} = 1;% �ڶ��� ���ý׶ε�ÿ��״̬�����ս׶ε�״̬������·��(����dist{i,2})


%% ����Ѱ��

% ��һ��ѭ�����������ÿһ���׶�
for i = stages-1:-1:1 % ���� ��Ϊ������׶��Ѿ���ʼ�����ӽ׶��Ŀ�ʼ
    num_states_f = length(nodes_dist{i, 1});  % ÿ���׶ε�״̬(��)

    % �ڶ���ѭ����������i�׶ε�ÿһ��״̬
    for j = 1:num_states_f
        num_states_r = length(nodes_dist{i+1, 1}); % ��i+1(��)�׶ε�״̬��      
        
        % ������ѭ����������i�׶εĵ�j��״̬����i+1�׶ε�ÿ(k)��״̬��ÿһ��·��
        for k = 1:num_states_r
            % ��:min{{d(B1,C1)+f(C1)},{d(B1,C2)+f(C2)},{d(B1,C3)+f(C2)}}
            if  nodes_dist{i,3}(j,k) + dist{i+1,2}(k,1) < dist{i,2}(j,1)% ��Ϊһ��ʼΪinf
                dist{i,2}(j,1) = nodes_dist{i,3}(j,k) + dist{i+1,2}(k,1);
                path{i, 2}(j,:) = [j, path{i+1, 2}(k,:)];% ����״̬��ӵ����׶�����·����ͷ
            end
        end
    end
end
            
%% �������
path_opt =  path(1,:);    
dist_opt =  dist{1,2};            
  