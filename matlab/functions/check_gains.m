%% check control gains
function check_gains(kd, kp, N, na)

    for i = 2:na
        if kd(i) <= 1/length(N{i}) || kd(i) <= 0
            fprintf('Kd%d (must be positive):\n', i)
            disp(kd(i))
            disp('Lower limit')
            disp(1/length(N{i}))
            error('Kd gains do not satisfy stability conditions')
        elseif kp(i) >= 4/length(N{i}) - 4/(kd(i)^2*length(N{i})^3) || kp(i) <= 0
            fprintf('Kp%d (must be positive):\n', i)
            disp(kp(i))
            disp('Upper limit')
            disp(4/length(N{i}) - 4/(kd(i)^2*length(N{i})^3))
            error('Kp gains do not satisfy stability conditions')

        end

    end

end