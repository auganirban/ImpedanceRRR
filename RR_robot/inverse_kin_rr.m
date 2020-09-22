function [sol1, sol2] = inverse_kin_rr(position)
    global l1; global l2;
    
    % Inverse Position Kinematics
    x = position(1);
    y = position(2);

    if -1 <= sqrt(x^2 + y^2 -l1 - l2)/(2*l1*l2) <= 1
        c2 = (x^2 + y^2 - (l1^2 + l2^2))/(2*l1*l2);
        s21 = sqrt(1 - c2^2);
        s22 = -sqrt(1 - c2^2);
        theta21 = atan2(s21,c2);
        theta22 = atan2(s22,c2);
        theta11 = atan2(y,x) - atan2(l2*sin(theta21), l1 + l2*cos(theta21));
        theta12 = atan2(y,x) - atan2(l2*sin(theta22), l1 + l2*cos(theta22));
        % Ensuring that theta1 lies between -pi and pi
        if (theta11 >= pi)
            theta11 = pi - theta11;
        end
        if (theta12 >= pi)
            theta12 = pi - theta12;
        end
        sol1 = [theta11; theta21];
        sol2 = [theta12; theta22];
    else
        error('Point is not reachable by manipulator');
    end
end