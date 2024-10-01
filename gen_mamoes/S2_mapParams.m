%% Has tunable parameters for agent properties

%% Domain bounds
maps.xmin = 0.0;
maps.xmax = 100.0; 
maps.ymin = 0.0;
maps.ymax = 100.0; 
maps.Lx = maps.xmax - maps.xmin;
maps.Ly = maps.ymax - maps.ymin;

maps.reflectThreshold = 2; 

xdel=1; %resolution in x
ydel=1; %resolution in y

xRange=maps.xmin:xdel:maps.xmax-xdel;
yRange=maps.ymin:ydel:maps.ymax-ydel;

[maps.X,maps.Y] = meshgrid(xRange,yRange);
Z = zeros(size(maps.X)); 

%% Create information maps for ergodic search
% The following presents examples of different maps and map generation 
% techniques. Pick whatever is apt for your test scenario. 

%% Example 1: Information map with three peaks
% m1=[50 50];     % Center of the peak        % [100 200];
% s1=25*eye(2);   % Variance in the peak      % 150*eye(2);
% m2=[100 90];    % Center of the peak        % [220 200]; 
% s2=100*eye(2);  % Variance in the peak      % 800*eye(2); 
% m3=[60 100];    % Center of the peak        % [120 120]; 
% s3=60*eye(2);   % Variance in the peak      % 600*eye(2); 
% 
% map1Temp = mvnpdf([maps.X(:), maps.Y(:)],m1,s1);
% map2Temp = mvnpdf([maps.X(:), maps.Y(:)],m2,s2);
% map3Temp = mvnpdf([maps.X(:), maps.Y(:)],m3,s3);
% mapTemp = (map1Temp + 3 * map2Temp + 2 * map3Temp); % Scalarized map
% mapTemp = max(mapTemp,0);               % crop below 0
% mapTemp = mapTemp./max(mapTemp);        % all values are brought to <=1
% mapTemp = mapTemp./sum(sum(mapTemp));   % Normalize
% 
% infoMap = mapTemp; 

%% Example 2: Information map with 10 almost delta peaks from the slides
% This example will help introduce information maps in the slides
% See slide deck Generalized_MAMOES Ver 6 from Sept 19th, 2024

peaks = [   30  35  25  25  35  70  65  75  65  75; 
            30  35  25  35  25  70  65  75  75  65;]';

pdfMap = zeros(numel(maps.X), 1); 
for i = 1:size(peaks,1)
    % close all; 
    m=peaks(i,:);
    s=30*eye(2);

    mapTemp = mvnpdf([maps.X(:), maps.Y(:)],m,s);
    pdfMap=(pdfMap + mapTemp);

    infoMap = i*pdfMap; 
    infoMap=max(infoMap,0);                 % crop below 0
    infoMap=infoMap./max(infoMap);          % all values are brought to <=1
    infoMap = infoMap./sum(sum(infoMap));   % Normalize
end

%% Example 3: Invoke maps using npy arrays from Akshaya's code in Python

% Note: maps in slides: 535, 536, 537, 538, 539 

% Uncomment to invoke the npy arrays as information maps
% maps.list = cell(700,1); 
% for i = 535:539
%     maps.list{i} = double(py.numpy.load(strcat(num2str(i),'.npy')));
% end

% This part of the code would go into the SA-MO-ES script
% map12Temp = map1Temp+map2Temp;                  % Equal weight
% map12Temp = map12Temp./max(map12Temp); 
% map12Temp = map12Temp./sum(sum(map12Temp)); 


% map3 = double(py.numpy.load(strcat(num2str(537),'.npy')));

% infoMap = map12Temp;
% drawMap = map12Temp;

%% Example 4: Extract a map from an image
% imgMap = im2double(rgb2gray(imread('535.jpg')));
% imgMap = imresize(imgMap,[maps.L(1), maps.L(2)]);
% imgMap =  imbinarize(imgMap);
% %imshow(img);
% infoMap = flip(imgMap);
% infoMap = infoMap./sum(sum(infoMap));
