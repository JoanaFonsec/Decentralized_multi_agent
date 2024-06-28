j=1;
i = 1;

         %Oval derivative / tangent

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
            if dista < abs(oval_tang(i,j))                                                  % Real measured distance to boundary
                oval_tang(i,j) = abs((oval_y - p{j}(i-1,1))/(oval_x - p{j}(i-1,2)));               
            end
        end

        %Circle derivative / tangent

        circle_tang(i,j) = (t_est.y(i,j) - p{j}(i-1,1))/(t_est.x(i,j) - p{j}(i-1,2)); 