function [] = visualize_robot_motion(q)
    global l1; global l2;
    
    end_pts = [zeros(2,1), [l1*cos(q(1)); l1*sin(q(1))], ...
               [l1*cos(q(1))+l2*cos(q(1)+q(2)); l1*sin(q(1))+l2*sin(q(1)+q(2))]];
    
    link1 = plot(end_pts(1,1:2), end_pts(2,1:2), "r", "linewidth", 6);
    xlim([-2,2]); ylim([-2,2]);
    hold on;
    link2 = plot(end_pts(1,2:3), end_pts(2,2:3), "g", "linewidth", 6);
    
    drawnow;
    
    delete(link1);
    delete(link2);
end
