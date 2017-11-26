% Close all open Serial Communication Ports 
clc
close all
clear all
delete(instrfindall);
%%

%% Initialize the Serial Port - setupSerial()
if (~exist('serialFlag','var'))
    [accelerometer.s,serialFlag]=setupSerial('COM13');
end

%%  Open a new Figure - add start/stop and close Serial button

% Initalize the figure that will be ploted in if it does not exist
if(~exist('h','var')||~ishandle(h))
    h=figure(1);
    ax=axes('box','on');
end

%%
if(~exist('button','var'))
    button=uicontrol('Style','pushbutton','String','Stop',...
        'pos',[0 0 50 25],'parent',h,...
        'Callback','quit','UserData',1);
end
%%

if(~exist('button2','var'))
    button2=uicontrol('Style','pushbutton','String','Close Serial Port',...
        'pos',[250 0 150 25],'parent',h,...
        'Callback','closeSerial','UserData',1);
end
%% Rolling Plot

buf_len=150;
index=1:buf_len;
index1=1:buf_len;
index2=1:buf_len;
index3=1:buf_len;
index4=1:buf_len;
index5=1:buf_len;
index6=1:buf_len;

rgdata=zeros(buf_len,1);
rgdata1=zeros(buf_len,1);
rgdata2=zeros(buf_len,1);
gxdata=zeros(buf_len,1);
gydata=zeros(buf_len,1);
gzdata=zeros(buf_len,1);

%%
while(get(button,'UserData'))
    % get the new values from the accelerometer
    [qW,qX,qY,qZ, gx, gy, gz, eulerx,eulery,eulerz] = readQuad(accelerometer);

    rg=dcm_gravity(eulerx,eulery,eulerz);          % Vector graphic

    
    acceleration=[gx gy gz];                       % Saving Accelerometer data in Array
    q=[qW,qX,qY,qZ];
                                                     % Gravity compensation  
    
    
    rgdata=[rgdata(2:end) ; rg(1)];
    rgdata1=[rgdata1(2:end);rg(2)];
    rgdata2=[rgdata2(2:end);rg(3)];

    
    gxdata=[gxdata(2:end) ; gx];                    % Array for moving Plot
    gydata=[gydata(2:end) ; gy];
    gzdata=[gzdata(2:end) ; gz];
    
    
    subplot(2,2,3);
    plot(index4,rgdata,'r')
    hold on;
    plot(index5,rgdata1,'bl')
    plot(index6,rgdata2,'g')
    hold off;
    axis([1 buf_len -10 10])
    xlabel('time');
    ylabel('Magnitude in m/s^2');
    grid minor;
    title(['Earth-Gravity Vector G´: ' num2str([rg(1) rg(2) rg(3)], 'X: %.2f  Y: %.2f  Z: %.2f')])
    
    %%
%     
% d_w=qW*qW;
% d_x=qX*qW;
% d_y=qY*qY;
% d_z=qZ*qZ;
% 
% ret_x=atan2d(2.0*(qX*qY+qZ*qW),(d_x-d_y-d_z+d_w));
% ret_y=asind(-2.0*(qX*qZ-qY*qW)/(d_x+d_y+d_z+d_w));
% ret_z=atan2d(2.0*(qY*qZ+qW*qW) , (-d_x-d_y+d_z+d_w));


    %%
    subplot(2,2,1);
    plot(index,gxdata,'r')
    hold on;
    plot(index1,gydata,'bl')
    plot(index2,gzdata,'g')
    hold off;
    grid minor;
    axis([1 buf_len -10 10]);
    xlabel('time');
    ylabel('Magnitude in m/s^2');
    title(['Accelerometer A: ' num2str([gx gy gz], 'X: %.2f  Y: %.2f  Z: %.2f')]);
        drawnow;
    
end


disp('Close Plot to End Session');
fclose(s); 
disp('Session Terminated...');