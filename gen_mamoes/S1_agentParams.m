%% Has tunable parameters for agent properties

%% Velocity Bounds, number of agents, 

robs.nagents = 1; % number of agents

robs.vlb =  0.1 * ones(robs.nagents,1); % Forward velocity lower bound
robs.vub =    2 * ones(robs.nagents,1); % Forward velocity upper bound
robs.wlb = -0.4 * ones(robs.nagents,1); % Forward angular vel lower bound
robs.wub =  0.4 * ones(robs.nagents,1); % Forward angular vel lower bound

%% Initializing agent locations

robs.initX = [60,20,25]; % For three agents - change appropriately
robs.initY = [30,80,25]; % For three agents - change appropriately
robs.initT = 120*pi/180*ones(robs.nagents,1); 

