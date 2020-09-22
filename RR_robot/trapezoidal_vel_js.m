function[q, dq, ddq, tm] = trapezoidal_vel_js(q_init, q_final, tf, dt)

% -----------------------------------------------------------------------|
% This code computes jointspace trajectory in position, velocity and acce|
% leration level. Equation 4.8 is used to calculate the jointspace       |
% trajectory at position level. Differentiating Equation 4.8 once and    |
% twice we can get the trajectory at the velocity and acceleration level.|
% Author: Anirban Sinha. Date: 06/11/2020.                               |
% -----------------------------------------------------------------------|

    gamma = 2; view_profiles = 1;
    dd_qc = gamma*4*(q_final - q_init)/(tf^2); % Siciliano pp-167

    % Compute tc's
    tc = zeros(size(q_final,1), 1);
    for i = 1:size(q_final,1)
        tc(i) = (0.5*tf) - 0.5*sqrt(((tf^2)*dd_qc(i) - 4*(q_final(i) - q_init(i)))/dd_qc(i));
    end
    
    % generate the time vector
    tm = 0:dt:tf;

    % Compute the joint position trajectory
    q = zeros(size(q_final,1), length(tm)); 
    dq = zeros(size(q_final,1), length(tm)); 
    ddq = zeros(size(q_final,1), length(tm));

    for i = 1:length(tm)
        for j = 1:size(q_final,1)
            if tm(i) <= tc(j)
                q(j,i) = q_init(j) + 0.5*dd_qc(j)*(tm(i)^2);
                dq(j,i) = dd_qc(j)*tm(i);
                ddq(j,i) = dd_qc(j);
            elseif (tm(i) > tc(j)) && (tm(i) <= tf -tc(j))
                q(j,i) = q_init(j) + dd_qc(j)*tc(j)*(tm(i) - 0.5*tc(j));
                dq(j,i) = dd_qc(j)*tc(j);
                ddq(j,i) = 0;
            else
                q(j,i) = q_final(j) - 0.5*dd_qc(j)*((tf - tm(i))^2);
                dq(j,i) = dd_qc(j)*(tf - tm(i));
                ddq(j,i) = -dd_qc(j);
            end
        end
    end
    
    if view_profiles
        for i = 1:size(q_final,1)
            figure(i)
            title(strcat("Profiles of joint",num2str(i)))
            % Plot the trajectory
            subplot(3, 1, 1)
            plot(tm, q(i,:))
            xlabel("time [s]")
            ylabel("q [rad]")
            grid on;

            subplot(3, 1, 2)
            plot(tm, dq(i,:))
            xlabel("time [s]")
            ylabel("dq [rad/s]")
            grid on;

            subplot(3, 1, 3)
            plot(tm, ddq(i,:))
            xlabel("time [s]")
            ylabel("ddq [rad/s^2]")
            grid on;
        end
    end

end