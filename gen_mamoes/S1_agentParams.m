%% Has tunable parameters for agent properties

%% Velocity Bounds, number of agents, 

robs.nagents = 1; % number of agents

robs.vlb =  0.1 * ones(robs.nagents,1); % Forward velocity lower bound
robs.vub =    2 * ones(robs.nagents,1); % Forward velocity upper bound
robs.wlb = -0.4 * ones(robs.nagents,1); % Forward angular vel lower bound
robs.wub =  0.4 * ones(robs.nagents,1); % Forward angular vel lower bound

%% Initializing agent locations

robs.initX = [60,20,25]; % DrB [70,20,25](slides) [140,20,25];
robs.initY = [30,80,25]; % DrB [20,80,25](slides) [20,120,125];
robs.initT = 120*pi/180*ones(robs.nagents,1);

