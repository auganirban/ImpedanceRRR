function [fe] = get_ee_force(xe,xr)
    % This function generates taskspace force whenever end-effector tip 
    % touches an elastic plane based on fe = Ke(xr - xe). The current 
    % implementation exerts force only along the x-axis of the base frame 
    % if there is a contact between the end-effector and the elastic plane.
    Ke = diag([1e4, 0]);
    if xe(1) > xr(1)
        fe = Ke*(xe - xr);
    else
        fe = zeros(2,1);
    end
end