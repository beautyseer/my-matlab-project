function result = segment_cuboid_intersection(p0, p1, corner, len, width, height)
% 判断线段与长方体是否相交
% 输入：
%   p0, p1: 线段的两个端点，格式为 [x y z]
%   corner: 长方体的基准角点，格式为 [x y z]
%   len, width, height: 长方体的尺寸（允许负数）
% 输出：
%   1-相交，0-不相交

% 计算长方体的六个边界
x_min = min(corner(1), corner(1) + len);
x_max = max(corner(1), corner(1) + len);
y_min = min(corner(2), corner(2) + width);
y_max = max(corner(2), corner(2) + width);
z_min = min(corner(3), corner(3) + height);
z_max = max(corner(3), corner(3) + height);

% 判断点是否在长方体内部
    function inside = is_inside(pt)
        inside = (pt(1) >= x_min && pt(1) <= x_max) && ...
                 (pt(2) >= y_min && pt(2) <= y_max) && ...
                 (pt(3) >= z_min && pt(3) <= z_max);
    end

% 如果任一端点在长方体内部
if is_inside(p0) || is_inside(p1)
    result = 1;
    return
end

% 初始化相交时间区间
t_enter = -inf;
t_exit = inf;

% X轴方向处理
dx = p1(1) - p0(1);
if abs(dx) < eps  % 处理除零情况
    if ~(p0(1) >= x_min && p0(1) <= x_max)
        result = 0;
        return
    end
else
    tx1 = (x_min - p0(1)) / dx;
    tx2 = (x_max - p0(1)) / dx;
    t_min_x = min(tx1, tx2);
    t_max_x = max(tx1, tx2);
    t_enter = max(t_enter, t_min_x);
    t_exit = min(t_exit, t_max_x);
    if t_enter > t_exit
        result = 0;
        return
    end
end

% Y轴方向处理
dy = p1(2) - p0(2);
if abs(dy) < eps
    if ~(p0(2) >= y_min && p0(2) <= y_max)
        result = 0;
        return
    end
else
    ty1 = (y_min - p0(2)) / dy;
    ty2 = (y_max - p0(2)) / dy;
    t_min_y = min(ty1, ty2);
    t_max_y = max(ty1, ty2);
    t_enter = max(t_enter, t_min_y);
    t_exit = min(t_exit, t_max_y);
    if t_enter > t_exit
        result = 0;
        return
    end
end

% Z轴方向处理
dz = p1(3) - p0(3);
if abs(dz) < eps
    if ~(p0(3) >= z_min && p0(3) <= z_max)
        result = 0;
        return
    end
else
    tz1 = (z_min - p0(3)) / dz;
    tz2 = (z_max - p0(3)) / dz;
    t_min_z = min(tz1, tz2);
    t_max_z = max(tz1, tz2);
    t_enter = max(t_enter, t_min_z);
    t_exit = min(t_exit, t_max_z);
    if t_enter > t_exit
        result = 0;
        return
    end
end

% 最终相交判断
if t_enter <= t_exit && t_enter <= 1 && t_exit >= 0
    result = 1;
else
    result = 0;
end
end