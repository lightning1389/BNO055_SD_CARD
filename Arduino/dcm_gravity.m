function [ rg] = dcm_gravity( alpha,beta,theta)
%  
% alpha=2;         % Alpha YAW -> Euler X
% beta=-85.8;         % Beta Pitch -> Eueler Y
% theta=4.2;         % Theta Roll -> Euler Z 

gravity=[0 0 9.81];
Rg=[0 0 0];
Ra=[0 0 0];
ma=[0 0 0];

% Rotation Matrix
r11=(cosd(alpha)*cosd(beta));
r12=(cosd(alpha)*sind(beta)*sind(theta) - sind(alpha)*cosd(theta));
r13=(cosd(alpha)*sind(beta)*cosd(theta)) + (sind(alpha)*sind(theta));
r21=sind(alpha)*cosd(beta);
r22=(sind(alpha)*sind(beta)*sind(theta)) + (cosd(alpha)*cosd(theta));
r23=(sind(alpha)*sind(beta)*cosd(theta)) - cosd(alpha)*sind(theta);
r31=-(1* sind(beta));
r32=cosd(beta) * sind(theta);
r33=cosd(beta) * cosd(theta);

% R Matrix

r_a=[cosd(alpha) -sind(alpha) 0 ; sind(alpha) cosd(alpha) 0 ; 0 0 1];
r_b=[cosd(beta) 0 sind(beta); 0 1 0 ; -sind(beta) 0 cosd(beta)];
r_t=[1 0 0 ; 0 cosd(theta) -sind(theta) ; 0 sind(theta) cosd(theta) ];


r_new = r_a*r_b*r_t;




RotationMatrix=[r11 r12 r13; 
                r21 r22 r23;
                r31 r32 r33];

% Determine of R Matrix
Rdet=det(RotationMatrix);             % Checking RotationMatrix,should be exactly +1
% fprintf('The Rotation Matrix is correct as the Determination of it is 1\n');



% alpha1=atand(r21/r11)                               %Yaw
% beta1=atand(-r31/(sqrt((r32.^2)+(r33.^2))))         %Pitch
% theta1=atand(r32/r33)                                

% alpha2=atand(r_new(2,1)/r_new(1,1))                               %Yaw
% beta2=atand(-r_new(3,1)/(sqrt((r_new(3,2).^2)+(r_new(3,3).^2))))         %Pitch
% theta2=atand(r_new(3,2)/r_new(3,3))                                





 rg(1)= gravity(1)*r11 + gravity(2)*r12 + gravity(3)*r13;
 rg(2)= gravity(1)*r21 + gravity(2)*r22 + gravity(3)*r23;
 rg(3)= gravity(1)*r31 + gravity(2)*r32 + gravity(3)*r33;


    

subplot(1,2,2);

cla;
hold on;
line(0,0,0, 'Color', 'bl','LineWidth',1,'Marker','o') 
    line(rg(1),rg(2),rg(3), 'Color', 'r','LineWidth',1,'Marker','o') 
    
    plot3([0 rg(1)],[0 rg(2)],[0 rg(3)],'color','bl')    
    title(['Orientation through Fusion Algorithm: ' num2str([alpha,beta,theta], 'Yaw: %.2f  Pitch: %.2f  Roll: %.2f ')])

        plot3([0 rg(1)],[0 0],[0 0],'color','r')    


            plot3([0 0],[0 rg(2)],[0 0],'color','r')    

            plot3([0 0],[0 0],[0 rg(3)],'color','r')    
hold off;
   % plot3([0 -5],[0 0],[0 0],'color','b')
   % plot3([0 0],[0 -5],[0 0],'color','b')
    
 %   plot3([0 0],[0 0],[0 5],'color','b')
    %plot3([-5 5],[0 0],[-5 -5],'color','r')
  %  plot3([][][])


view(3)

%     line(rg(1),rg(2),0, 'Color', 'r','LineWidth',1,'Marker','o') 
%     line(0,0,rg(3), 'Color', 'r','LineWidth',1,'Marker','o') 
% 
%     
%     %plot3([0 rg(1)],[0 rg(2)],[0 rg(3)])
%     plot3([0 5],[0 0],[0 0])
%     plot3([0 0],[0 5],[0 0])
%     plot3([0 0],[0 0],[0 5])

%%
 %   plot3([0 rg(1)],[0 rg(2)],[0 rg(3)])
    axis([-10 10 -10 10 -10 10])
         xlabel('Vertical')
         ylabel('longitudinal')
         zlabel('lateral')
    grid on

   
    
    
end



    