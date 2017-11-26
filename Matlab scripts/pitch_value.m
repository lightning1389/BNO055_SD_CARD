%% Script to calculate the pitch value based on the raw accelerometer readings
%   After a moving average the filter is 
%   load('sample_data_set.mat')         % Loading a dataset

%% Smoothing around 150 samples 10 Seconds.

x=smooth(ACCELERATIONX_A,150);
y=smooth(ACCELERATIONY_A,150);
z=smooth(ACCELERATIONZ_A,150);


%% 
n=length(ACCELERATIONX_L);     % Getting length of one string
i=1;

while (i<n)
    i=i+1;
    %Pitch(i)=asind((ACCELERATIONX_A(i))/(sqrt((ACCELERATIONY_A(i)).^2+(ACCELERATIONZ_A(i)).^2))); 
    x1=x(i)/sqrt(y(i).^2+z(i).^2);
    y1=z(i)/sqrt(y(i).^2+z(i).^2);
   
    
    Pitch(i)=atan2d(x1,y1);         % 
end



