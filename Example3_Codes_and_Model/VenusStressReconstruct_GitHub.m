%%

Num_Interface = 1;                                  % Number of interfaces.
t = xout{1}.Values.Time;                            % Total time steps
PoI = [0.200000000000000	1.06000000000000	-0.810000000000000];           % Point of interest, as defined before. If different from the PoI in the reconstruction file, the actual positions inquired will be the node positions closest to the PoI.

% Find the node positions closest to the PoI.

NodesCoord = model.Mesh.Nodes;
for i = 1:1:size(PoI,1)
    NodePosition = PoI(i,:);
    XX = NodesCoord(1,:)-NodePosition(1);
    YY = NodesCoord(2,:)- NodePosition(2);
    ZZ = NodesCoord(3,:)- NodePosition(3);
    NodeIndex(i) = intersect(intersect(find(abs(XX)<0.02),find(abs(YY)<0.02)),find(abs(ZZ)<0.02));
end
NodePosition = NodesCoord(:,NodeIndex)';



%% Pre-define the size of the variables.

time = zeros(1,length(t));
StressZZ = zeros(size(PoI,1),length(time));
StressXX = zeros(size(PoI,1),length(time));
StressYY = zeros(size(PoI,1),length(time));
StressYZ = zeros(size(PoI,1),length(time));
StressXZ = zeros(size(PoI,1),length(time));
StressXY = zeros(size(PoI,1),length(time));
DisX = zeros(1,length(time));
DisY = zeros(1,length(time));
DisZ = zeros(1,length(time));

%%
% Break the total time steps into number of segNum pieces, and deal with them one at a time so the
% post-processing does not consume all the ram.

segNum = 500;
segment = cell(segNum,1);
startId = 1;
for i = 2:1:segNum-1
    segment{i,1} = round((length(t)-startId)*(i-1)/segNum)+startId+1:round((length(t)-startId)*i/segNum)+startId;
end
segment{1,1} = startId:round((length(t)-startId)/segNum)+startId;
segment{end,1} = round((length(t)-startId)*(segNum-1)/segNum)+startId+1:length(t);


%% To find whole THS at certain points

% Reconstruct the full order solutions

for abcd = 1:1: size(segment,1)                                 % Post-process one piece of time steps during each loop.
    Interval = segment{abcd,1};
    BodyFrameSolu = zeros(size(Interval,2),6);                  % Relative displacements, velocities and accelerations of the FFR origin to itself, must be zeros for all the six DoFs.

    interfaceDofSolu = [BodyFrameSolu xout{19}.Values.Data(Interval,1:(Num_Interface-1)*6)];    % Inferface DoFs solutions. xout{19} is the state solution of the reduced order flexible solid.
    internDofSolu =  xout{19}.Values.Data(Interval,(Num_Interface-1)*6+1:end);                  % Internal DoFs solutions.
    simSolution = [interfaceDofSolu internDofSolu];

    interfaceDofSoluDot =  [BodyFrameSolu xout{20}.Values.Data(Interval,1:(Num_Interface-1)*6)];% The first derivative of the inferface DoFs solutions. xout{20} is the first derivative of the state solution of the reduced order flexible solid.
    internDofSoluDot =  xout{20}.Values.Data(Interval,(Num_Interface-1)*6+1:end);               % The first derivative of the internal DoFs solutions.
    simSolutionDot = [interfaceDofSoluDot internDofSoluDot];

    u = simSolution';
    ut = simSolutionDot';
    utt = zeros(size(u,1),size(u,2));                           % The second derivative of the DoFs. Not needed for reconstructing displacement and velocity solutions, but syntax requires it so all zeros with the compatible size.
    tlist = t(Interval);
    RTrom = reconstructSolution(R,u,ut,utt,tlist);              % Run the reconstruction function.


    % strain = evaluateStrain(RTrom);
    %stress = evaluateStress(RTrom);                             % Evaluate the stress.
    %time(1,Interval-startId+1) = tlist;
    % for aq = 1:1:length(NodeIndex)
    %     StressZZ(aq,Interval-startId+1) = stress.zz(NodeIndex(aq),:);
    %     StressXX(aq,Interval-startId+1) = stress.xx(NodeIndex(aq),:);
    %     StressYY(aq,Interval-startId+1) = stress.yy(NodeIndex(aq),:);
    %     StressXY(aq,Interval-startId+1) = stress.xy(NodeIndex(aq),:);
    %     StressXZ(aq,Interval-startId+1) = stress.xz(NodeIndex(aq),:);
    %     StressYZ(aq,Interval-startId+1) = stress.yz(NodeIndex(aq),:);
    % end

    % The following is for construct principal stress map. It finds the
    % maximum sigma1 sigma3 and sigma with largest magnitude over the abcd time interval at
    % each mesh node.  The output is maxsigma1 maxsigma3 and maxsigma.


    % AllPstress = evaluatePrincipalStress(RTrom);
    % sigma1 = AllPstress.s1  ;
    % sigma3 = AllPstress.s3  ;
    %
    % [~,timeInstance]=max(abs(sigma1(:,:)),[],2);
    % for shui = 1:1:size(timeInstance)
    %     MaxMax(shui) = sigma1(shui,timeInstance(shui));
    % end
    % maxsigma1(:,abcd) = MaxMax';
    %
    % [~,timeInstance]=max(abs(sigma3(:,:)),[],2);
    % for shui = 1:1:size(timeInstance)
    %     MaxMax(shui) = sigma3(shui,timeInstance(shui));
    % end
    % maxsigma3(:,abcd) = MaxMax';
    %
    %
    %
    % Maxsigma =  maxsigma1(:,abcd);
    % a4 = abs(maxsigma1(:,abcd))-abs(maxsigma3(:,abcd));
    % changeId = find(a4<0);
    % Maxsigma(changeId) = maxsigma3(changeId,abcd);
    % maxsigma(:,abcd) = Maxsigma;
    

    % The following find the maximum tensile principal stress and maximum
    % compressive principal stress over the time interval abcd. 

    AllPstress = evaluatePrincipalStress(RTrom);
    sigma1 = AllPstress.s1  ;
    sigma3 = AllPstress.s3  ;

    for i =1:1:size(sigma1,1)
        Sigma1 = sigma1(i,:);
        if isempty(Sigma1(Sigma1 > 0))
            max_tensile(i) = NaN;
        else
            max_tensile(i) = max(Sigma1(Sigma1 > 0));
        end
    end

    for i =1:1:size(sigma3,1)
        Sigma3 = sigma3(i,:);
        if isempty(Sigma3(Sigma3 < 0))
            max_compressive(i) = NaN;
        else
            max_compressive(i) = min(Sigma3(Sigma3 < 0));
        end
    end

    Max_tensile(:,abcd) = max_tensile';
    Max_compressive(:,abcd) = max_compressive';

end


%% For cross-section stress plot


Interval = [22326:9:22865];


%%
BodyFrameSolu = zeros(size(Interval,2),6);                  % Relative displacements, velocities and accelerations of the FFR origin to itself, must be zeros for all the six DoFs.

interfaceDofSolu = [BodyFrameSolu xout{19}.Values.Data(Interval,1:(Num_Interface-1)*6)];    % Inferface DoFs solutions. xout{19} is the state solution of the reduced order flexible solid.
internDofSolu =  xout{19}.Values.Data(Interval,(Num_Interface-1)*6+1:end);                  % Internal DoFs solutions.
simSolution = [interfaceDofSolu internDofSolu];

interfaceDofSoluDot =  [BodyFrameSolu xout{20}.Values.Data(Interval,1:(Num_Interface-1)*6)];% The first derivative of the inferface DoFs solutions. xout{20} is the first derivative of the state solution of the reduced order flexible solid.
internDofSoluDot =  xout{20}.Values.Data(Interval,(Num_Interface-1)*6+1:end);               % The first derivative of the internal DoFs solutions.
simSolutionDot = [interfaceDofSoluDot internDofSoluDot];

u = simSolution';
ut = simSolutionDot';
utt = zeros(size(u,1),size(u,2));                           % The second derivative of the DoFs. Not needed for reconstructing displacement and velocity solutions, but syntax requires it so all zeros with the compatible size.
tlist = t(Interval);
RTrom = reconstructSolution(R,u,ut,utt,tlist);              % Run the reconstruction function.
%%
load("queryPoints.mat")
x = xnew;    % The x should cover the x coordinates range of the cross-section plane
y = ynew;    % The y just need to be where the vertical cross section at. In this case, it is the y of the CoM.
z = znew;    % The z should cover the z coordinates range of the cross-section plane

% Create a regular grid covering your data
nx = 300; nz = 300;   % Resolution
xgv = linspace(min(x), max(x), nx); % mesh grid is a must for surf
zgv = linspace(min(z), max(z), nz); %
[Xg, Zg] = meshgrid(xgv, zgv);

intrpStress = interpolateStress(RTrom,Xg,y(1,1)*ones(nx,nz),Zg); % Notice this actually covers a rectanglur section of Xg by Zg, which is larger than the actual cross section. But this is fine, as points too far away from the geometry will be treated as no stress.

% Calculate principal stresses if needed
% StressXX = intrpStress.xx;
% StressYY = intrpStress.yy;
% StressZZ = intrpStress.zz;
% StressXY = intrpStress.xy;
% StressYZ = intrpStress.yz;
% StressXZ = intrpStress.xz;
%
%
% sigma1 = zeros(size(StressXX,1),length(Interval));
% sigma3 = zeros(size(StressXX,1),length(Interval));
% for ii = 1:1:length(Interval)
% for i = 1:1:size(StressXX,1)
%         stress_tensor = [StressXX(i,ii) StressXY(i,ii) StressXZ(i,ii);
%            StressXY(i,ii) StressYY(i,ii) StressYZ(i,ii);
%            StressXZ(i,ii) StressYZ(i,ii) StressZZ(i,ii)];
%         sigma = eig(stress_tensor);
%         sigma1(i,ii) = max(sigma);
%         sigma3(i,ii) = min(sigma);
% end
% end

maxTensile = max(intrpStress.zz);
[Maxmax,timeInstance] = max(maxTensile)

%% Plot the cross section stress distribution over the defined time interval
i = 1;
for ii = 1:1:timeInstance
    figure(i);
    Instance = timeInstance-i+1;
    surf(Xg,Zg,reshape(intrpStress.zz(:,Instance),size(Xg)),"LineStyle","none")
    xlim([-0.4 0.4])
    ylim([-1,1.1])
    view(0, 90)
    colormap(turbo(1024));
    clim([-15e6 15e6])
    i = i+1;
end








