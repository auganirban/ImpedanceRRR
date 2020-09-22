function [] = visualize_motion_rrr(q,fig_handle)
    global l1; global l2; global l3;
% % %     global filename; global iter;
    
    end_pts = [zeros(2,1), [l1*cos(q(1)); l1*sin(q(1)),], ...
               [l1*cos(q(1))+l2*cos(q(1)+q(2)); l1*sin(q(1))+l2*sin(q(1)+q(2))],...
               [l1*cos(q(1))+l2*cos(q(1)+q(2))+l3*cos(q(1)+q(2)+q(3)); l1*sin(q(1))+l2*sin(q(1)+q(2))+l3*sin(q(1)+q(2)+q(3))]];
    
    link1 = plot(end_pts(1,1:2), end_pts(2,1:2), "r", "linewidth", 6);
    % xlim([-2,2]); ylim([-2,2]);
    xlim([-0.5,2.3]); ylim([-0.5,2.3]);
    hold on;
    link2 = plot(end_pts(1,2:3), end_pts(2,2:3), "g", "linewidth", 6);
    link3 = plot(end_pts(1,3:4), end_pts(2,3:4), "b", "linewidth", 6);
    
    drawnow;
    
% % %     % Capture the plot as an image 
% % %     frame = getframe(fig_handle); 
% % %     im = frame2im(frame);
% % %     [imind,cm] = rgb2ind(im,256);
% % %     
% % %     % Write to the GIF File
% % %     gif_iter = iter-1;
% % %     if gif_iter == 1 
% % %       imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
% % %     elseif (mod(gif_iter,10) == 0)
% % %       imwrite(imind,cm,filename,'gif','WriteMode','append'); 
% % %     end 
    
    delete(link1);
    delete(link2);
    delete(link3);
end
