%% INIT
clear; 
close all; clc;

% Inititalization target

t.x = 50*ones(500,1);
t.y = 40*ones(500,1);
t.r1 = 25*ones(500,1);
t.r2 = 15*ones(500,1);

n=5;

for j = 1:n
    t_est.x(1,j) = t.x(1);
    t_est.y(1,j) = t.y(1);
    t_est.r(1,j) = t.r1(1); %the bigger radius in the beginning
end

% Inititalization agents

cars = cars_setup;
z1(1,:)                      = [cars(1).x0, cars(1).y0, cars(1).v0, cars(1).psi0]; 
z2(1,:)                      = [cars(2).x0, cars(2).y0, cars(2).v0, cars(2).psi0];
z3(1,:)                      = [cars(3).x0, cars(3).y0, cars(3).v0, cars(3).psi0]; 
z4(1,:)                      = [cars(4).x0, cars(4).y0, cars(4).v0, cars(4).psi0];
z5(1,:)                      = [cars(5).x0, cars(5).y0, cars(5).v0, cars(5).psi0];
p = {z1 z2 z3 z4 z5};

% Initialization distances
Distance = zeros(990,n);
Db = radar(1,t,p,n);
Distance(1,:)=Db; 

oval_tang = 999*ones(500,n);
circle_tang = zeros(500,n);

%% Set up the movie.
writerObj = VideoWriter('out.avi'); % Name it.
writerObj.FrameRate = 10; % How many frames per second.
open(writerObj); 

for i = 2:500
  
    hold off; plot(0,0); hold on;
     
    % Compute new target positions
    t.x(i) = t.x(i-1) + 0.1*(randn + 0.2);      
    t.y(i) = t.y(i-1) + 0.1*(randn + 0.2);
    t.r1(i) = t.r1(i-1) + 0.1*(randn + 0.2);
    t.r2(i) = t.r2(i-1) + 0.1*(randn + 0.2);
    
    %% Calculations

    % Distances to agent j
    Db = radar(i,t,p,n);
    Distance(i,:) = Db; 
    
    %% Estimates of c and r by LSQ
    
    for j = 1:n    
        %Minimization of a circle for vehicle j using 3 vehicles (j, j-1, and j+1)
        c1=j;
        c2=j-1;
        c3=j+1;
        if j==1  %for vehicle 1, the car before is n
            c2=n;
        elseif j==n  %for vehicle n, the car after is 1
            c3=1;            
        end
        
        fun = @(x) (norm(x(1:2) - p{c1}(i-1,1:2)) - (x(3)+Db(c1)))^2 +(norm(x(1:2) - p{c2}(i-1,1:2)) - (x(3)+Db(c2)))^2 +(norm(x(1:2) - p{c3}(i-1,1:2)) - (x(3)+Db(c3)))^2;
        x0 = [t_est.x(i-1,j),t_est.y(i-1,j),t_est.r(i-1,j)];
        A = -eye(3);
        b = [0;0;0];
        err(i-1,:) = fmincon(fun,x0,A,b);

        %Estimates of each circle t_est for each vehice j
        t_est.x(i,j) = err(i-1,1);
        t_est.y(i,j) = err(i-1,2);
        t_est.r(i,j) = err(i-1,3);     
        
        %Derivatives of estimates. To be used in the position function
         t_est.dotx(i,j) = t_est.x(i,j) - t_est.x(i-1,j);
         t_est.doty(i,j) = t_est.y(i,j) - t_est.y(i-1,j);
         t_est.dotr(i,j) = t_est.r(i,j) - t_est.r(i-1,j); 
                 
         %% For plotting comparison of estimate and real
         
         %Real radius(theta) for the point where agent j is
         Radius(i-1,j) = norm([t.x(i-1) t.y(i-1)] - p{j}(i-1,1:2),2) - Db(j); 
           
         %Oval derivative / tangent
    
        oi = 999;
        
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
            if dista <= oi                                                  % Real measured distance to boundary
                oi = dista;                                                               % Real measured distance to boundary
                next_beta = beta + pi/50;
                next_oval_x = t.x(i-1) + signal_x*t.r1(i-1)*t.r2(i-1)*(t.r2(i-1)*t.r2(i-1)+t.r1(i-1)*t.r1(i-1)*tan(next_beta)*tan(next_beta)).^(-1/2);  %the point x,y on the oval
                next_oval_y = t.y(i-1) + signal_y*t.r2(i-1)*( 1 - ((next_oval_x-t.x(i-1))/t.r1(i-1)).^2 ).^(1/2);
                %                 oval_tang = [next_oval_x next_oval_y] - [oval_x oval_y]; 
                oval_tang = [oval_y next_oval_x] - [next_oval_y oval_x];   % The perpendicular one
            end
        end

        %Circle derivative / tangent

        circle_tang = [t_est.x(i,j) t_est.y(i,j)] - [p{j}(i-1,1) p{j}(i-1,2)]; 
        
        %cos(alpha) = <u,v> / (|u|.|v|)
        
        the_cosine(i,j) = dot(circle_tang,oval_tang) / (norm(oval_tang)*norm(circle_tang));

    end
    
    %% Position
        
    p = position(i,t_est,p,Db);
    plot_environment(n,i,p,cars,t,t_est);
    frame = getframe(gcf); %, [90 90 300 300]); % 'gcf' can handle if you zoom in to take a movie.
    writeVideo(writerObj, frame);
    
    pause(0.5);
end

hold off
close(writerObj);

