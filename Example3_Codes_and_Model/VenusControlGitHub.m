% Control code for the Simulink model: Venus
% Author: Zheng-You Zhang

% Mass in kg.
% Length in m. 
% Time in s. 

clear all
clc

numContact = 4;                                           % Number of contact feet

g = 9.80665;                                                % Gravitational acceleration
E = 60e09;                                                  % Material Young's modulus in Pa.
rho = 2700;                                                 % Material density.
nu = 0.2;                                                   % Poisson's ratio.
ms = 591.086;


% Global properties
maS = ms;                                             


% Support material, according to the museum website, it marble. 
v = 0.2;
Width = 2*(0.2034+0.05);
Depth = Width;
G = 25e09;
rho_support = 2700;

% Stiffness and damping for one vertical support element.
Ksupport_total = G*Width/(2*(1-v))*(3.1*(Depth/Width)^0.75+1.6);
Csupport_total = 2*sqrt(G/rho_support)*rho_support*Depth*Width;
kk2=Ksupport_total/numContact;                  %Stiffnes for one vertical element.
cc1=Csupport_total/numContact;                  %Damping for one vertical element.



% Stiffness and damping for one horizontal frictional element.
muf = 0.625;%
k_n=kk2;                                        %Stiffnes for one frictional element.
c_n=cc1;                                        %Damping for one frictional element.
                     
%Initial conditions

hc = 0;
zc = 1*(hc/2 - (maS)*g/(Ksupport_total));                   % Initial deformation of one vertical spring.

%% Generate geometry

gm = importGeometry('Venus de Milo Low Res v2.step');
s = 7.6176; 
gm = scale(gm,s);
BottomCenter = [-0.0114534, 1.05977, -0.975217];     
addVertex(gm,"Coordinates",BottomCenter)
model = createpde('structural','modal-solid');
model.Geometry = gm;


pdegplot(model,'VertexLabels','on','FaceAlpha',0.5);                                

%Specify structural properties

structuralProperties(model,"YoungsModulus",E, ...
    "PoissonsRatio",nu, ...
    "MassDensity",rho);


% Create finite-element mesh

msh=generateMesh(model,"GeometricOrder","linear","Hmin",0.02,Hmax=1);

nodeID = findNodes(msh,"nearest",BottomCenter');
origins = [msh.Nodes(1,nodeID), msh.Nodes(2,nodeID), msh.Nodes(3,nodeID)];
numFrames = size(origins,1);

structuralBC(model, ...
    'Face',[87,66,281,428,280,279], ...
    'Constraint','multipoint', ...
    'Reference',origins);


model.SolverOptions.MaxShift = 500;




%% Modal analysis
modalresults = solve(model,FrequencyRange=[0,5e4]); % in rad/s. for original Youngs, 5e4.
Freq = modalresults.NaturalFrequencies;


%% Apply CB reduction

 
FreqRange = [0 Freq(60)+0.2];  

R = reduce(model,"FrequencyRange",FreqRange);  % The frequency rang is in rad/s.

Reduced.K = (R.K+R.K')/2;                       % Reduced stiffness matrix.
Reduced.M = (R.M+R.M')/2;                       % Reduced mass matrix.
Reduced.P = R.ReferenceLocations';



%% Save the variables needed for full order resconstruction 

filename = 'Venus-Reconstruction-60Modes.mat';
save(filename,'R');


%% Define simulation parameters


                           
t_end =22;                                              % Total simulation time.                                           
relTol = 1e-4;                                          % Solver relative tolerance.
absTol = 1e-4;                                          % Solver absolute tolerance.


