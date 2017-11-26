function [s,flag] = setupSerial(comPort)

% Initialize the serial port communication between arduino and matlab
% we ensure that the arduino is also communicationg with matlab at this
% time
% a predifined code on the arduino acknowledges this.
%if setup is complete then the value of setup is returned as 1 else 0 

flag=1;

s=serial(comPort);
set(s,'DataBits',8);
set(s,'StopBits',1);
set(s,'BaudRate',115200);
set(s,'Parity','none');
fopen(s);
a='b';
while(a~='a')
    a=fread(s,1,'uchar');
end
if(a=='a')
    disp('serial read');
end
fprintf(s,'%c','a');
mbox=msgbox('Serial Communication Setup-'); uiwait(mbox);
fscanf(s,'%u');

end
