%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Author: Bhaskar Vundurthy(bvundurthy@outlook.com)
%%Based on the works of Elif Ayvali (eayvali@gmail.com) / Hadi Salman (hadicsalman@gmail.com)
%%Biorobotics lab, The Robotics Institute, Carnegie Mellon University
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Run the parameters file - prevents data transfer between functions
close all; clear variables; clc;
tic

run("S1_agentParams.m"); 
run("S2_mapParams.m"); 
run("S3_ergOptParams.m"); 

% Need infoMap, robs, maps, ergs, opts, figs to execute the following

%% Obtain the fourier coefficients of the information map in infoMap

Nagents = robs.nagents; 

X = maps.X; 
Y = maps.Y; 
Lx = maps.Lx; 
Ly = maps.Ly; 

Nkx = ergs.Nkx; 
Nky = ergs.Nky; 

mu=reshape(infoMap,size(X));
mu = mu/sum(sum(mu));% normalize information distribution
mu = reshape(mu,size(X)); 
muk=zeros(Nkx, Nky); % Initializing Fourier coefficents of infoMap
% Our implementation to compute fourier coefficients HK and muk
HK = sqrt(Lx*Ly) .* [1 sqrt(1/2)*ones(1,Nky-1)]' * [1 sqrt(1/2)*ones(1,Nkx-1)];
for kx = 0:Nkx-1
    for ky = 0:Nky-1
        muk(kx+1, ky+1) = sum(sum( mu .* cos(kx * pi * X/Lx) .* cos(ky * pi *  Y/Ly) )) / HK(ky+1,kx+1);
    end
end

%% Simulation: Construct the trajectory using feedback laws from the paper
Nsteps = opts.Nsteps;
dt = opts.dt;

% Initializing Fourier coefficients of the trajectory
Ck = zeros(Nkx, Nky);

vlb = robs.vlb; 
vub = robs.vub; 
wlb = robs.wlb;
wub = robs.wub; 

pose.x = robs.initX; 
pose.y = robs.initY; 
pose.t = robs.initT; 

xmin = maps.xmin;
ymin = maps.ymin;
xmax = maps.xmax;
ymax = maps.ymax;

KX = ergs.KX;
KY= ergs.KY;
LK = ergs.LK;

kdOBJ = KDTreeSearcher([X(:),Y(:)]); % to eval traj cost by closest point
ergodicity = zeros(Nsteps,1);
traj = zeros(Nsteps, Nagents, 3);

for it = 1:Nsteps
    currTime = (it) * dt;
    
    %% SMC_Update rule to compute the next location for each robot
    %Calculating the fourier coefficients of time average statistics distribution 
    for iagent = 1:Nagents 
        xrel = pose.x(iagent) - xmin;
        yrel = pose.y(iagent) - ymin;
        Ck = Ck + cos(KX * pi * xrel/Lx) .* cos(KY * pi * yrel/Ly) * dt ./ HK';
    end
    
    % Use the fourier coefficients of trajectory to compute the heading direction and speed
    traj_stat = zeros(size(mu(:)));    
    for iagent = 1:Nagents
        xrel = pose.x(iagent) - xmin;
        yrel = pose.y(iagent) - ymin;
        
        Bjx = sum(sum(LK./ HK' .* (Ck - Nagents*currTime*muk) .* (-KX *pi/Lx .*sin(KX * pi * xrel/Lx) .* cos(KY *pi * yrel/Ly))));
        Bjy = sum(sum(LK./ HK' .* (Ck - Nagents*currTime*muk) .* (-KY *pi/Ly .*cos(KX * pi * xrel/Lx) .* sin(KY *pi * yrel/Ly))));
    
        GammaV =  Bjx*cos(pose.t(iagent)) + Bjy*sin(pose.t(iagent));
        GammaW = -Bjx*sin(pose.t(iagent)) + Bjy*cos(pose.t(iagent));
        
        % Updating agent location based on SMC feedback control law
        if GammaV >= 0
            v = vlb(iagent);
        else
            v = vub(iagent);
        end
        
        if GammaW >= 0
            w = wlb(iagent);
        else
            w = wub(iagent);
        end
        
        %velocity motion model
        if(abs(w) < 1e-10 )
            pose.x(iagent) = pose.x(iagent) + v*dt*cos(pose.t(iagent));   
            pose.y(iagent) = pose.y(iagent) + v*dt*sin(pose.t(iagent));
        else
            pose.x(iagent) = pose.x(iagent) + v/w*(sin(pose.t(iagent) + w*dt) - sin(pose.t(iagent)));   
            pose.y(iagent) = pose.y(iagent) + v/w*(cos(pose.t(iagent)) - cos(pose.t(iagent)+ w*dt));    
        end
        pose.t(iagent) = pose.t(iagent)+ w*dt; 
        
        % Reflect the agent if it hits the boundaries
        agentx = pose.x(iagent); agenty = pose.y(iagent); 
        if agentx < (xmin)
            agentx = xmin + (xmin - agentx);
        end
        if agentx > (xmax)
            agentx = xmax - (agentx - xmax);
        end
        if agenty < (ymin)
            agenty = ymin + (ymin - agenty);
        end
        if agenty > (ymax)
            agenty = ymax - (agenty - ymax);
        end
        pose.x(iagent) = agentx; pose.y(iagent) = agenty; 

        %% Saving the trajectory and computing the ergodicity

        traj(it,iagent,1)= pose.x(iagent);
        traj(it,iagent,2)= pose.y(iagent);

        % Obtaining time average statistics distribution of the trajectory
        % distcrete pdf of the trajectory is same size as information map
        
        currTraj = reshape(traj(:,iagent,1:2),Nsteps,2); 
        ind = knnsearch(kdOBJ, currTraj(:,1:2));
        currTrajStat = accumarray(ind,1,size(mu(:)));
        currTrajStat = currTrajStat./sum(sum(currTrajStat)); % normalizing

        traj_stat = traj_stat + currTrajStat;         
    end
    ck = Ck/Nagents/currTime; 
    ergodicity(it) =  sum(sum( LK .* (ck - muk).^2 ));
    if (mod(it,10)==0)
        fprintf("Iteration: %d \n", it);
    end
end

toc
%% Figures, animations and videos

%% Plot utility 
if figs.image
    figure; 
    % set(gcf,'color','w'); 
    hold on
    surface(X,Y,Z,reshape(infoMap,size(X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
    axis tight; 
    axis equal; 
    % surface(X,Y,Z,reshape(drawMap,size(X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
    % colorbar;
    
    if ~figs.color
        colormap('gray'); 
    end
        
    % Plot agent initial and final positions along with the trajectory
    colors = flip(ff2n(3)); % Neat way of creating a binary rep of numbers 0-7 as a vector
    for iagent = 1:Nagents
        scatter(robs.initX(iagent), robs.initY(iagent),'filled','MarkerFaceColor',colors(iagent+1,:));
        if ~figs.animate
            plot(traj(1:Nsteps,iagent,1),traj(1:Nsteps,iagent,2),'Color',colors(iagent+1,:), 'LineWidth', 2);
        end
        scatter(traj(Nsteps,iagent,1), traj(Nsteps,iagent,2),'filled', 'Marker', 's', 'MarkerFaceColor',colors(iagent+1,:));
    end
end

if (figs.video && figs.animate)
    v = VideoWriter('peaks.mp4', 'MPEG-4'); % DrB
    open(v);  %DrB
end

if figs.animate
    for it = 1:Nsteps
        for iagent = 1:Nagents
            if( mod(it,10) == 0)
                plot(traj(1:it,iagent,1),traj(1:it,iagent,2),'Color',colors(iagent+1,:), 'LineWidth', 2);
                pause(0.005); 
                if figs.video
                    frame = getframe(gcf); %DrB
                    writeVideo(v,frame); %DrB
                end
            end
        end
    end
end

if figs.ergodicity
    time=dt:dt:dt*Nsteps;
    figure;
    loglog(time(1:end),ergodicity(1:end))
    % axis([0.001 5 0.0001,1])
    xlabel('Time (sec)');
    ylabel('Ergodicity');
    % ylabel('Coverage Metric, \phi(t)');
    title('metric of ergodicity as a function of time')
end
