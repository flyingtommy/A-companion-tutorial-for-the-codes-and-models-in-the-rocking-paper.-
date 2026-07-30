% This code is the reconstruction of 3d deformation and global displacement of the statue in
% global reference frame. The final ouput variables are DeformGlobal and
% GlobalDisp.

Num_Interface = 1;                                  % Number of interfaces.
t = xout{1}.Values.Time;                            % Total time steps
PoI = [-0.2171	0.96443	1.063246];
FFRCenterPos = [-0.0114534, 1.05977, -0.975217];


% Find the node positions closest to the PoI.

NodesCoord = model.Mesh.Nodes;
for i = 1:1:size(PoI,1)
    NodePosition = PoI(i,:);
    XX = NodesCoord(1,:)-NodePosition(1);
    YY = NodesCoord(2,:)- NodePosition(2);
    ZZ = NodesCoord(3,:)- NodePosition(3);
    NodeIndex(i) = intersect(intersect(find(abs(XX)<0.002),find(abs(YY)<0.002)),find(abs(ZZ)<0.002));
end
NodePosition = NodesCoord(:,NodeIndex)';



%% Pre-define the size of the variables.

time = zeros(1,length(t));
DeformX = zeros(1,length(time)); % Deformation relative to the body reference position in the floating frame direction
DeformY = zeros(1,length(time));
DeformZ = zeros(1,length(time));

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

    for aq = 1:1:length(NodeIndex)
        DeformX(aq,Interval-startId+1) = RTrom.Displacement.x(NodeIndex(aq),:);
        DeformY(aq,Interval-startId+1) = RTrom.Displacement.y(NodeIndex(aq),:);
        DeformZ(aq,Interval-startId+1) = RTrom.Displacement.z(NodeIndex(aq),:);
    end
end



%% Deformaiton and total displacement in global x y z

DeformGlobal = zeros(3,length(t));% Deformation relative to the body reference position in the global direction
PositionGlobal = zeros(3,length(t)); % Position vector at the PoI in the global direction
for i = 1:1:length(t)
    thetax = logsout{17}.Values.Data(i,1);
    thetay = logsout{17}.Values.Data(i,2);
    thetaz = logsout{17}.Values.Data(i,3);

    Rz = [cos(thetaz) -sin(thetaz) 0;sin(thetaz) cos(thetaz) 0;0 0 1];
    Ry = [cos(thetay) 0 sin(thetay); 0 1 0;-sin(thetay) 0 cos(thetay)];
    Rx = [1 0 0;0 cos(thetax) -sin(thetax);0 sin(thetax) cos(thetax)];
    DeformGlobal(:,i) = Rz*Ry*Rx*[DeformX(1,i);DeformY(1,i);DeformZ(1,i)];

    PositionGlobal(:,i) = DeformGlobal(:,i)+ Rz*Ry*Rx*(NodePosition'-FFRCenterPos')+[logsout{18}.Values.Data(i);logsout{19}.Values.Data(i);logsout{20}.Values.Data(i)];
end

%If free vibration, then the above Position(:,1) is the position vector at
%initial release, which is not the actual initial posiiton vector at rest.
% Need to find the initial position
if logsout{17}.Values.Data(1,2)~=0

    thetax = logsout{17}.Values.Data(1,1);
    thetay = logsout{17}.Values.Data(1,2);
    thetaz = logsout{17}.Values.Data(1,3);

    Rz = [cos(thetaz) -sin(thetaz) 0;sin(thetaz) cos(thetaz) 0;0 0 1];
    Ry = [cos(thetay) 0 sin(thetay); 0 1 0;-sin(thetay) 0 cos(thetay)];
    Rx = [1 0 0;0 cos(thetax) -sin(thetax);0 sin(thetax) cos(thetax)];

    Rtotal = Rz*Ry*Rx;

    InitialPosVec = NodePosition'-FFRCenterPos' + transpose(Rtotal)*[logsout{18}.Values.Data(1);logsout{19}.Values.Data(1);logsout{20}.Values.Data(1)];
    GlobalDisp = PositionGlobal - InitialPosVec; % Total displacement (i.e., relative to the global frame position) at the PoI in the global direction

else
    GlobalDisp = PositionGlobal - PositionGlobal(:,1); % Total displacement (i.e., relative to the global frame position) at the PoI in the global direction
end

