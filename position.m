function z = position(i,t,z,D)
%% Variables

    a = 3;
    n = 5;              % n agents
    psi = ones(n,2);    % bearing
    dfront = ones(n,1);   % distance between two agents c1 & c3
    dback = ones(n,1);   % distance between two agents c1 & c2
        
%% Calculate vehicle reference

    for j = 1:n 
         psi(j,:) = (1/abs(D(j)))*([t.x(i,j) t.y(i,j)]-[z{j}(i-1,1) z{j}(i-1,2)]);  % Calculate psi (bearing) of car j for target of car j
        v1 = psi(j,:);
        
        % Vehicle j using 3 vehicles (j, j-1, and j+1)
        c1=j;
        c2=j-1;
        c3=j+1;
        if j==1  %for vehicle 1, the car before is n
            c2=n;
        elseif j==n  %for vehicle n, the car after is 1
            c3=1;            
        end
       
        % Calculate Distance to front (See which is the vehicle imediately ahead)
     
        dfront(j) = norm([z{c1}(i-1,1) z{c1}(i-1,2)]-[z{c3}(i-1,1) z{c3}(i-1,2)]); 
        
        % Calculate Distance to back (See which is the vehicle imediately before)
        
        dback(j) =  norm([z{c1}(i-1,1) z{c1}(i-1,2)]-[z{c2}(i-1,1) z{c2}(i-1,2)]); 
        
        
%         % Calculate Beta (See which is the vehicle imediately ahead)
%      
%         v2 = (1/D(c3))*([t.x(i,c1) t.y(i,c1)]-[z{c3}(i-1,1) z{c3}(i-1,2)]);  % Calculate psi (bearing) of car c3 for target of car j or c1
%         angle = atan2d(v2(2),v2(1)) - atan2d(v1(2),v1(1));
%         if angle <= 0
%             angle = angle + 360;
%         end
% 
%         beta(j) = deg2rad(angle);
%         
%         % Calculate BackBeta (See which is the vehicle imediately before)
% 
%         v2 = (1/D(c2))*([t.x(i,c1) t.y(i,c1)]-[z{c2}(i-1,1) z{c2}(i-1,2)]);  % Calculate psi (bearing) of car c2 for target of car j or c1
%         angle = atan2d(v2(2),v2(1)) - atan2d(v1(2),v1(1));
%         if angle <= 0
%             angle = angle + 360;
%         end
%         backbeta(j) = deg2rad(angle);


        % Beta as distance between two agents
        
        beta(j) = norm([z{c1}(i-1,1:2) z{c3}(i-1,1:2)],2);
        backbeta(j) = norm([z{c1}(i-1,1:2) z{c2}(i-1,1:2)],2);
        
        %Calculate the velocity
%         z{j}(i,6) = t.dotx(i,j) + (D(j) - t.r(i,j) - t.dotr(i,j))*psi(j,1) + (a + beta(j))*psi(j,2); 
%         z{j}(i,7) = t.doty(i,j) + (D(j) - t.r(i,j) - t.dotr(i,j))*psi(j,2) - (a + beta(j))*psi(j,1);
        z{j}(i,6) = D(j)*psi(j,1) + a*psi(j,2); 
        z{j}(i,7) = D(j)*psi(j,2) - a*psi(j,1);        
%         z{j}(i,6) = D(j)*psi(j,1) + a*(beta(j)/backbeta(j))*psi(j,2); 
%         z{j}(i,7) = D(j)*psi(j,2) - a*(beta(j)/backbeta(j))*psi(j,1);        
        
        
%         if sqrt(z{j}(i,6)*z{j}(i,6)+z{j}(i,7)*z{j}(i,7)) >= 0.5    %Up to 2 of speed
%             z{j}(i,6) = z{j}(i,6) *2/ sqrt(z{j}(i,6)*z{j}(i,6)+z{j}(i,7)*z{j}(i,7));
%             z{j}(i,7) = z{j}(i,7) *2/ sqrt(z{j}(i,6)*z{j}(i,6)+z{j}(i,7)*z{j}(i,7));
%         end

    total_v=sqrt(z{j}(i,6)*z{j}(i,6)+z{j}(i,7)*z{j}(i,7));
        
        %if sqrt(z{j}(i,6)*z{j}(i,6)+z{j}(i,7)*z{j}(i,7)) >= 1    %Up to 2 of speed
            z{j}(i,6) = z{j}(i,6) *2*(beta(j)/backbeta(j))/ total_v;
            z{j}(i,7) = z{j}(i,7) *2*(beta(j)/backbeta(j))/ total_v;
        %end

%         if z{j}(i,6) - z{j}(i-1,6) > 1    %Up to 1 of acceleration
%             z{j}(i,6) = z{j}(i-1,6) +1;
%         end    
%         if z{j}(i,7) - z{j}(i-1,7) > 1    %Up to 1 of acceleration
%             z{j}(i,7) = z{j}(i-1,7) +1;
%         end    
%         
%         if z{j}(i,6) - z{j}(i-1,6) < -1    %Up to 1 of acceleration
%             z{j}(i,6) = z{j}(i-1,6) -1;
%         end    
%         if z{j}(i,7) - z{j}(i-1,7) < -1    %Up to 1 of acceleration
%             z{j}(i,7) = z{j}(i-1,7) -1;
%         end    
        
        %Calculate next position
        z{j}(i,1) = z{j}(i-1,1) + z{j}(i,6);    %This is rotated
        z{j}(i,2) = z{j}(i-1,2) + z{j}(i,7);  
        z{j}(i,3) = sqrt((z{j}(i,1)-z{j}(i-1,1))^2 + (z{j}(i,2)-z{j}(i-1,2))^2);   %Total velocity
        if (z{j}(i,1)-z{j}(i-1,1)) < 0
            flag = 1;
        else
            flag = 0;
        end
        z{j}(i,4) = atan(((z{j}(i,2)-z{j}(i-1,2))/(z{j}(i,1)-z{j}(i-1,1))))+flag*pi;  %Car angle
        z{j}(i,5) = dfront(j)/dback(j);
    end
        
%       fprintf('Angles %.2f %.2f %.2f %.2f\n',beta);
end