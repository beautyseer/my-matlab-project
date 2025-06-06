function q_final = move_along_y(arm, q_init, dy)
    q = q_init(:);
    max_iterations = 100;
    step_size = 0.1;
    tolerance = 1e-6;

    T_init = arm.fkine(q);
    R = t2r(T_init);
    p_target = transl(T_init) + R(:,2) * dy;  % 末端Y方向

    for i = 1:max_iterations
        T_curr = arm.fkine(q);
        p_curr = transl(T_curr);
        e = p_target - p_curr;

        if norm(e) < tolerance
            break;
        end

        J = arm.jacob0(q);   % 世界坐标系雅可比
        J_pos = J(1:3, :);
        dq = pinv(J_pos) * (e * step_size);
        q = q + dq(:);
    end

    arm.plot(q);
    q_final = q;
end
