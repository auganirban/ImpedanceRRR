function [xe, dxe, ddxe] = trajectory_ts(q, dq, ddq)
    % This function maps jointspace trajectories to taskspace trajectories
    for i = 1:length(q)
        xe(:,i) = frd_kin_rr(q(:,i));
        dxe(:,i) = jacobian_rr(q(:,i))*dq(:,i);
        ddxe(:,i) = jacobian_rr(q(:,i))*ddq(:,i) + diff_jacobian_rr(q(:,i), dq(:,i))*dq(:,i);
    end
end