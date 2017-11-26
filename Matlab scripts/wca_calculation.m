%% Translating filtered data into an array of WCA 
%   Input:
%   x - x-axis acceleration vector 
%   y - y-axis acceleration vector 
%   Output: 
%   wca gives an array of the calculated wind correction angles to the cor-
%   responding vector x and y.
%   Note: Here the direction is not yet determined. 
% 
% 
%   Author: Ilhan Dogan
%   Date: 01.02.2016
%   Version 1.0


function wca= wca_calculation(x,y)

n=length(x);        % Dertmine the lenght of the array to count iterations
i=0;
while(i<n)          
    i=i+1;
    xneu=abs(x(i));    % Using only absoulute values for the formula
    yneu=abs(y(i));
    new_value=xneu/sqrt(xneu^2+yneu^2); % Formula 5-2 in master's thesis
    wca(i)=acosd(new_value);            %   Result in array after acos 
    
end

end
