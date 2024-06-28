function cars = cars_setup

xi = [29.4 79 70.9 29.4 11.1];
yi = [14.1 20 55.4 66.9 45.4];
di = [0 pi/2 0 2*pi/3 0];

% model parameters
car1.a_max                = 0.5;                        % acceleration limit
car1.beta_max             = pi/2;                     % side slip angle limit
car1.beta_dot_max         = (pi/180)*(200);           % side slip angle rate limit
car1.l_f                  = 2;                        % distance between center of gravity and front axle
car1.l_r                  = 2;                        % distance between center of gravity and rear axle
car1.vehicle_width        = 4;                        % vehicle width
car1.Ts                   = 0.01;                      % sampling time (both of MPC and simulated vehicle)
car1.nstates              = 4;                        % number of states
car1.ninputs              = 2;                        % number of inputs

% environment parameters
car1.activate_obstacles   = 0;                        % 0 if there are no obstacle, 1 otherwise
car1.obstacle_centers     = [10 -2; 20  0; 30 -2; 40 6]; % x and y coordinate of the 4 obstacles
car1.obstacle_size        = [2 6];                    % size of the obstacles
car1.lane_semiwidth       = 400;                      % semi-width of the lane

% control parameters
car1.controller           = 'MPC';                   % 'SF' for state-feedback controller, 'MPC' for MPC controller

% simulation parameters
car1.x0                   = xi(1);                   % initial x coordinate
car1.y0                   = yi(1);                   % initial y coordinate
car1.v0                   = 2;                      % initial speed
car1.psi0                 = di(1);                   % initial heading angle
car1.N_max                = 500;                     % maximum number of simulation steps

car2 = car1;
car2.x0                   = xi(2);                    
car2.y0                   = yi(2);   
car2.psi0                 = di(2);                   % initial heading angle

car3 = car1;
car3.x0                   = xi(3);                    
car3.y0                   = yi(3);   
car3.psi0                 = di(3);                   % initial heading angle\

car4 = car1;
car4.x0                   = xi(4);                    
car4.y0                   = yi(4);   
car4.psi0                 = di(4);                   % initial heading angle

car5 = car1;
car5.x0                   = xi(5);                    
car5.y0                   = yi(5);   
car5.psi0                 = di(5);                   % initial heading angle

cars = [car1; car2; car3; car4; car5];
end