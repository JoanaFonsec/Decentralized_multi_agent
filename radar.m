function Db = radar(i,t,p,n)

Db = 999*ones(1,n);

if i==1   % Calculate D of each vehicle in the first step
    for j = 1:n                                                  
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
            oval_x = t.x(1) + signal_x*t.r1(1)*t.r2(1)*(t.r2(1)*t.r2(1)+t.r1(1)*t.r1(1)*tan(beta)*tan(beta)).^(-1/2);  %the point x,y on the oval
            oval_y = t.y(1) + signal_y*t.r2(1)*( 1 - ((oval_x-t.x(1))/t.r1(1)).^2 ).^(1/2);
            dista = norm([oval_x oval_y] - p{j}(1,1:2),2);                       % Car to boundary
            if dista < abs(Db(j))                                                  % Real measured distance to boundary
                Sigma = sign( ((p{j}(1,1)-t.x(1))/t.r1(1)).^2 + ((p{j}(1,2)-t.y(1))/t.r2(1)).^2 - 1);   % Real inside (-1) or outside (1) of circle   
                Db(j) = dista*Sigma;               
            end
        end  
    end
else
    for j = 1:n                                                  
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
            oval_x = t.x(i-1) + signal_x*t.r1(i-1)*t.r2(i-1)*(t.r2(i-1)*t.r2(i-1)+t.r1(i-1)*t.r1(i-1)*tan(beta)*tan(beta)).^(-1/2);  %the point x,y on the oval
            oval_y = t.y(i-1) + signal_y*t.r2(i-1)*( 1 - ((oval_x-t.x(i-1))/t.r1(i-1)).^2 ).^(1/2);
            dista = norm([oval_x oval_y] - p{j}(i-1,1:2),2);                       % Car to boundary
            if dista < abs(Db(j))                                                  % Real measured distance to boundary
                Sigma = sign( ((p{j}(i-1,1)-t.x(i-1))/t.r1(i-1)).^2 + ((p{j}(i-1,2)-t.y(i-1))/t.r2(i-1)).^2 - 1);   % Real inside (-1) or outside (1) of circle   
                Db(j) = dista*Sigma;               
            end
        end  
    end
 end

end