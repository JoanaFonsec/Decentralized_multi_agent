%% INIT
close all; clc;

i=1;
j=1;

oval_x = zeros(100,1);
oval_y = zeros(100,1);

for beta = 0:pi/50:2*pi
    if beta>pi/2 && beta< 3*pi/2  %signal for tangent
        signal_x = -1;
    else
        signal_x = 1;
    end
    if beta>0 && beta< pi  %signal for tangent
        signal_y = 1;
    else
        signal_y = -1;
    end
    i = fix(50*beta/pi+1);
    oval_x(i,1) = t.x(1) + signal_x*t.r1(1)*t.r2(1)*(t.r2(1)*t.r2(1)+t.r1(1)*t.r1(1)*tan(beta)*tan(beta)).^(-1/2);  %the point x,y on the oval
    oval_y(i,1) = t.y(1) + signal_y*t.r2(1)*( 1 - ((oval_x(i,1)-t.x(1))/t.r1(1)).^2 ).^(1/2);
%     dista = norm([oval_x oval_y] - p{j}(1,1:2),2);                       % Car to boundary
%     if dista < abs(Db(j))                                                  % Real measured distance to boundary
%         Sigma = sign( ((p{j}(1,1)-t.x(1))/t.r1(1)).^2 + ((p{j}(1,2)-t.y(1))/t.r2(1)).^2 - 1);   % Real inside (-1) or outside (1) of circle   
%         Db(j) = dista*Sigma;               
%     end
end  


% th = 0:pi/50:2*pi;
% xt = t.r1(i) * cos(th) + t.x(i);
% yt = t.r2(i) * sin(th) + t.y(i);
% plot(xt, yt, 'LineWidth', 2)
% hold on
plot(oval_x, oval_y, 'LineWidth', 3)
