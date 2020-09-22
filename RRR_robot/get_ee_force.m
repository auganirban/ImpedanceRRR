function [fe] = get_ee_force(xe,xr1,xr2,obs1,obs2)
    global phase;
    % This function generates taskspace force whenever end-effector tip 
    % touches an elastic plane based on fe = Ke(xr - xe). The current 
    % implementation exerts force only along the x-axis of the base frame 
    % if there is a contact between the end-effector and the elastic plane.
    
% % %     obstacle1.active_side1.vertices = [obs1.origin',[obs1.origin(1);obs1.origin(2)+obs1.height]];
% % %     obstacle1.active_side1.w = [obstacle1.active_side1.vertices(2,1)-obstacle1.active_side1.vertices(2,2);obstacle1.active_side1.vertices(1,2)-obstacle1.active_side1.vertices(1,1)];
% % %     obstacle1.active_side1.w = obstacle1.active_side1.w/norm(obstacle1.active_side1.w);
% % %     obstacle1.active_side1.b = -obstacle1.active_side1.w'*obstacle1.active_side1.vertices(:,1);
% % %     obstacle1.active_side1.d = obstacle1.active_side1.w'*xe(1:2,1)+obstacle1.active_side1.b;
% % %     fprintf("Obs1_Side1: %2.4f\n",obstacle1.active_side1.d);
% % %     obstacle1.active_side2.vertices = [[obs1.origin(1);obs1.origin(2)+obs1.height],[obs1.origin(1)+obs1.width;obs1.origin(2)+obs1.height]];
% % %     obstacle1.active_side2.w = [obstacle1.active_side2.vertices(2,1)-obstacle1.active_side2.vertices(2,2);obstacle1.active_side2.vertices(1,2)-obstacle1.active_side2.vertices(1,1)];
% % %     obstacle1.active_side2.w = obstacle1.active_side2.w/norm(obstacle1.active_side2.w);
% % %     obstacle1.active_side2.b = -obstacle1.active_side2.w'*obstacle1.active_side2.vertices(:,1);
% % %     obstacle1.active_side2.d = obstacle1.active_side2.w'*xe(1:2,1)+obstacle1.active_side2.b;
% % %     fprintf("Obs1_Side2: %2.4f\n",obstacle1.active_side2.d);
% % %     
% % %     obstacle2.active_side1.vertices = [obs2.origin',[obs2.origin(1);obs2.origin(2)+obs2.height]];
% % %     obstacle2.active_side1.w = [obstacle2.active_side1.vertices(2,1)-obstacle2.active_side1.vertices(2,2);obstacle2.active_side1.vertices(1,2)-obstacle2.active_side1.vertices(1,1)];
% % %     obstacle2.active_side1.w = obstacle2.active_side1.w/norm(obstacle2.active_side1.w);
% % %     obstacle2.active_side1.b = -obstacle2.active_side1.w'*obstacle2.active_side1.vertices(:,1);
% % %     obstacle2.active_side1.d = obstacle2.active_side1.w'*xe(1:2,1)+obstacle2.active_side1.b;
% % %     fprintf("Obs2_Side1: %2.4f\n",obstacle2.active_side1.d);
% % %     obstacle2.active_side2.vertices = [obs2.origin',[obs2.origin(1)+obs2.width;obs2.origin(2)]];
% % %     obstacle2.active_side2.w = [obstacle2.active_side2.vertices(2,1)-obstacle2.active_side2.vertices(2,2);obstacle2.active_side2.vertices(1,2)-obstacle2.active_side2.vertices(1,1)];
% % %     obstacle2.active_side2.w = obstacle2.active_side2.w/norm(obstacle2.active_side2.w);
% % %     obstacle2.active_side2.b = -obstacle2.active_side2.w'*obstacle2.active_side2.vertices(:,1);
% % %     obstacle2.active_side2.d = obstacle2.active_side2.w'*xe(1:2,1)+obstacle2.active_side2.b;
% % %     fprintf("Obs2_Side2: %2.4f\n",obstacle2.active_side2.d);
% % %     
% % %     % Figure out end-effector force
% % %     d1 = obstacle1.active_side1.d;
% % %     d2 = obstacle1.active_side2.d;
% % %     d3 = obstacle2.active_side2.d;
% % %     
% % %     if xe(2) > obs1.origin(2) 
% % %         if (d1<0 && d2>0 && d3>0)
% % %             fe(1) = -Ke*d1;
% % %             fe(2) = -mu*fe(1);
% % %         end
% % %     end
    
    if phase == 1
        if xe(2) < xr1(2)
            phase = 2;
        end
    end
    
    Ke = diag([1e6, 0]); mu = 0.1;
    if (xe(1) > xr1(1)) && (phase == 1) 
        if (xe(2) >= xr1(2)) || (xe(2) <= xr2(2))
            fe = Ke*(xe(1:2) - xr1);
            fe(2) = -mu*fe(1);
        else
            fe = zeros(2,1);
        end
    elseif (xe(2) > xr1(2)) && (phase == 2)
        fe = [mu*Ke(1,1)*(xe(2)-xr1(2));Ke(1,1)*(xe(2)-xr1(2))];
    elseif (xe(2) < xr2(2)) && (phase == 2)
        fe = [mu*Ke(1,1)*(xr2(2)-xe(2));-Ke(1,1)*(xr2(2)-xe(2))];
    else
        fe = zeros(2,1);
    end
end