function isCollision = checkRobotSelfCollision(jointPositions, linkRadius)
% 极简版六轴机械臂自碰撞检测函数
% 输入：
%   jointPositions - 各关节三维坐标，7x3矩阵（7个关节点，每个点[x,y,z]）
%   linkRadius - 连杆半径（假设所有连杆半径相同）
% 输出：
%   isCollision - 是否发生碰撞，true/false

% 默认无碰撞
isCollision = false;

% 检查连杆线段之间的碰撞
numJoints = size(jointPositions, 1);
for i = 1:numJoints-1
    A = jointPositions(i,:);    % 连杆起点
    B = jointPositions(i+1,:);  % 连杆终点
    
    for j = i+2:numJoints-1
        C = jointPositions(j,:);    % 另一连杆起点
        D = jointPositions(j+1,:);  % 另一连杆终点
        
        % 计算两线段间的最短距离
        [dis, ~, ~, ~, ~] = distance_between_lines(A, B, C, D);
        
        % 如果距离小于两倍连杆半径，发生碰撞
        if dis < (2 * linkRadius)
            isCollision = 1;
            return;
        end
    end
end

end

% 示例使用
