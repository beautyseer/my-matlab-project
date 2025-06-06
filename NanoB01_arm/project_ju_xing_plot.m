function [xy_project,xz_project,yz_project] = project_ju_xing_plot(vertex, length, width, height)
    % 输入参数：
    % vertex: 长方体的一个顶点坐标，例如 [x0, y0, z0]
    % length: 长方体的长度（沿 x 轴）
    % width: 长方体的宽度（沿 y 轴）
    % height: 长方体的高度（沿 z 轴）
    % 输出为[左下角坐标，右上角坐标]
    % 提取顶点坐标
    x0 = vertex(1);
    y0 = vertex(2);
    z0 = vertex(3);

    % 计算长方体的 8 个顶点坐标
    vertices = [
        x0, y0, z0;
        x0 + length, y0, z0;
        x0 + length, y0 + width, z0;
        x0, y0 + width, z0;
        x0, y0, z0 + height;
        x0 + length, y0, z0 + height;
        x0 + length, y0 + width, z0 + height;
        x0, y0 + width, z0 + height
    ];

    % 计算投影到 xy 平面的左下角和右上角坐标（忽略 z 坐标）
    xy_proj = vertices(:, 1:2);
    xy_min = min(xy_proj); % 左下角
    xy_max = max(xy_proj); % 右上角

    % 计算投影到 xz 平面的左下角和右上角坐标（忽略 y 坐标）
    xz_proj = vertices(:, [1, 3]);
    xz_min = min(xz_proj); % 左下角
    xz_max = max(xz_proj); % 右上角

    % 计算投影到 yz 平面的左下角和右上角坐标（忽略 x 坐标）
    yz_proj = vertices(:, 2:3);
    yz_min = min(yz_proj); % 左下角
    yz_max = max(yz_proj); % 右上角

    % 输出投影结果
%     disp('投影到 xy 平面的左下角和右上角坐标:');
%     disp([xy_min; xy_max]);
%     disp('投影到 xz 平面的左下角和右上角坐标:');
%     disp([xz_min; xz_max]);
%     disp('投影到 yz 平面的左下角和右上角坐标:');
%     disp([yz_min; yz_max]);
    xy_project = [xy_min,xy_max];
    xz_project = [xz_min,xz_max];
    yz_project = [yz_min,yz_max];

    %绘制长方体
    
    hold on;
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('长方体及其投影');

    % 绘制长方体
    faces = [
        1, 2, 3, 4;  % 底面
        5, 6, 7, 8;  % 顶面
        1, 2, 6, 5;  % 前面
        2, 3, 7, 6;  % 右面
        3, 4, 8, 7;  % 后面
        4, 1, 5, 8   % 左面
    ];
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'cyan', 'FaceAlpha', 0.5);

    % 绘制投影
%     plot3([xy_min(1), xy_max(1), xy_max(1), xy_min(1), xy_min(1)], ...
%           [xy_min(2), xy_min(2), xy_max(2), xy_max(2), xy_min(2)], ...
%           zeros(1, 5), 'r', 'LineWidth', 2); % xy 平面投影
%     plot3([xz_min(1), xz_max(1), xz_max(1), xz_min(1), xz_min(1)], ...
%           zeros(1, 5), ...
%           [xz_min(2), xz_min(2), xz_max(2), xz_max(2), xz_min(2)], 'g', 'LineWidth', 2); % xz 平面投影
%     plot3(zeros(1, 5), ...
%           [yz_min(1), yz_max(1), yz_max(1), yz_min(1), yz_min(1)], ...
%           [yz_min(2), yz_min(2), yz_max(2), yz_max(2), yz_min(2)], 'b', 'LineWidth', 2); % yz 平面投影

    % 添加图例
%     legend('长方体', 'xy 平面投影', 'xz 平面投影', 'yz 平面投影');
%     hold off;
end