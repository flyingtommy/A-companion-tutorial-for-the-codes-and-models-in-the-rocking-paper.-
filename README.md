# A-accompany-code-for-the-rocking-paper
Provides the codes for the cases studied in the paper

![Figure description](images/3d.JPG)

## Time Segmentation

<font color="red">my name is tom</font>

The script divides the simulation time vector into `segNum` equal parts.
Each segment contains the indices of `t` belonging to that range.

```matlab
segNum = 100;
segment = cell(segNum,1);

for i = 2:segNum-1
    segment{i} = round((length(t)-startId)*(i-1)/segNum) + startId : ...
                 round((length(t)-startId)*i/segNum) + startId;
end

segment{1}   = startId : round((length(t)-startId)/segNum) + startId;
segment{end} = round((length(t)-startId)*(segNum-1)/segNum)+startId : length(t);
```
fewafewa

# Tutorial: Reduced-Order Flexible-Body Modeling in Simulink

## Overview
This tutorial provides the MATLAB codes, Simulink models, and the necessary explanations to run the examples described in the companion paper. Two examples are included:

1. A realistic column-like flexible structure  
2. A frame structure used in a physical experiment  

These examples demonstrate how to build and simulate reduced-order flexible solids in Simulink using Craig–Bampton reduction and the Floating Frame of Reference (FFR) formulation.

---

## Pre-requisites

Readers should have a strong understanding of:

- Craig–Bampton reduction  
- Floating Frame of Reference (FFR) formulation  

### Suggested Readings
- The companion paper introducing both theories  
- A reference providing a clear introduction to the Craig–Bampton method, including verification checks and reduction tips  
- Chapters in book “?” describing the FFR formulation  
- The official MATLAB Craig–Bampton example to become familiar with MATLAB’s workflow for reduced-order flexible solids in Simulink  

---

## Why This Documentation?

MATLAB already includes a Craig–Bampton example, but this tutorial is necessary for two reasons.

### 1. Additional functionalities
This tutorial extends MATLAB’s standard implementation with:

- The ability to retain any desired fixed-interface modes (MATLAB's example only keeps modes within one frequency band)  
- Full-order reconstruction of deformation, strain, stress, and global displacement from Simulink results (MATLAB’s example only returns global interface displacements)  

### 2. Applicability beyond rocking problems
MATLAB’s example is focused on a rocking-block demonstration and does not cover:

- The use of OxContact elements (an in-house toolbox for contact modeling in rocking problems)  
- The combination of reduced-order elements with general structural dynamics problems  

Both examples in this tutorial address these limitations.

---

## Summary

This tutorial provides:

- A practical workflow for reduced-order flexible-body modeling  
- Improvements over MATLAB’s default example  
- Guidance for applications beyond rocking-block problems  
- Tools to reconstruct full-field mechanical quantities from reduced-order simulations  

By the end of this tutorial, users will understand how to build reduced models, implement them in Simulink, and perform full-field post-processing.

1.	Steps for Craig-Bampton reduction in matlab
a.	Define the parameters for the structure and other common parameters
```matlab
NumInterFace = 4;
StiModi = 1;

g = 9.80665;
Width = 2.088;
Depth = 2.088*1.5;
E = 5.42538e09;
rho = 2541.09;
nu = 0.3;
L = 26.609;
%L = 1;
A = Width*Depth;
ms = rho*L*A;
CrossSize = [Width/2 Depth/2; -Width/2 Depth/2; -Width/2 -Depth/2; Width/2 -Depth/2];

% Define the length of tetrahedral model and the beam model
L_te =L;
Subdi = 10; % Subdivide the beam element into 10, although there are only 8 beam elements. The base is
% replaced with the tetraheral element which is 20% height.
Le = L/Subdi;

% Global properties
p = 1;
maS = ms;               %Total mass of the system.
hcg = L/2;
alphacg = atan((Width/2)/hcg);


% Stiffness and damping for one horizontal frictional element.
muf = 0.8;
fh = 40; %21                %Normally from 10 to 100 hz.
wN=2*pi*fh;
ksin=0.5;                 %Damping ratio.

k_n=(maS)*wN^2/NumInterFace/StiModi;           %Stiffnes for one frictional element.
c_n=ksin*2*sqrt(k_n*NumInterFace*(maS))/NumInterFace;      %Damping for one frictional element.

% Stiffness and damping for one vertical support element.
fv = 40;                 %Normally from 10 to 100 hz.
wn = 2*pi*fv;
dR = 0.5;
Ksupport_total = (wn^2)*(maS)/StiModi;%Sqrt(k/m) gives rad/s
Csupport_total = dR*2*sqrt(Ksupport_total*(maS));
kk2=Ksupport_total/NumInterFace;   %Stiffnes for one vertical element.
cc1=Csupport_total/NumInterFace;   %Damping for one vertical element.

%Initial conditions
hc = 0;
zc = hc/2 - (maS)*g/(Ksupport_total);
```


b.	Generate or import geometry of the component for which the craig-bampton reduction will be needed. Details can found in this [example]( https://uk.mathworks.com/help/sm/ug/model-excavator-dipper-arm.html). 
```matlab
gm = multicuboid(Width,Depth,L_te);
distanceToEdge = 0.2;
LF_coords = [-Width/2+distanceToEdge*Width/2;-Depth/2+distanceToEdge*Depth/2;0];
RF_coords = [Width/2-distanceToEdge*Width/2;-Depth/2+distanceToEdge*Depth/2;0];
LB_coords = [-Width/2+distanceToEdge*Width/2;Depth/2-distanceToEdge*Depth/2;0];
RB_coords = [Width/2-distanceToEdge*Width/2;Depth/2-distanceToEdge*Depth/2;0];


RF_dis = RF_coords - LF_coords;
LB_dis = LB_coords - LF_coords;
RB_dis = RB_coords - LF_coords;



origins = [LF_coords';RF_coords';LB_coords';RB_coords'];
numFrames = size(origins,1);
addVertex(gm,"Coordinates",origins);


PoI = [-Width/2 0 0.1*L_te;-Width/2 0 0.2*L_te;-Width/2 0 0.3*L_te;-Width/2 0 0.4*L_te;-Width/2 0 0.5*L_te
    -Width/2 0 0.6*L_te;-Width/2 0 0.7*L_te;-Width/2 0 0.8*L_te;-Width/2 0 0.9*L_te;-Width/2 0 L_te
    Width/2 0 0.1*L_te;Width/2 0 0.2*L_te;Width/2 0 0.3*L_te;Width/2 0 0.4*L_te;Width/2 0 0.5*L_te;Width/2 0 0.6*L_te
    Width/2 0 0.7*L_te;Width/2 0 0.8*L_te;Width/2 0 0.9*L_te;Width/2 0 L_te];

for i = 1:1:size(PoI,1)
    addVertex(gm,"Coordinates",PoI(i,:));
end

model = createpde('structural','modal-solid');
model.Geometry = gm;

pdegplot(model,'EdgeLabels','on','FaceAlpha',0.5);
```

c.	Mesh the component. This is where the position of interface node is decide. Mesh first time to know position of the nearby vertex, then add the vertex and mesh again. Add vertex type multipoint constrains. The multipoint constrain is needed because tetrahedral elements node only has three dofs, however an interface should have six dofs in Simulink, so the multipoint constrain essentially forms a small rigid plane that has six dofs. 
```matlab
%Specify structural properties and boundary conditions
structuralProperties(model,"YoungsModulus",E, ...
    "PoissonsRatio",nu, ...
    "MassDensity",rho);
% Create finite-element mesh
hmax = 0.5;%5;0.8
%msh=generateMesh(model,GeometricOrder="quadratic",Hmax=hmax);
%msh2d=generateMesh(model2d,GeometricOrder="quadratic",Hmax=hmax);
%msh=generateMesh(model,Hmax=hmax,GeometricOrder="linear");

msh=generateMesh(model,"Hface",{[1 2],0.05},Hmax=hmax,GeometricOrder="linear",Hgrad=1.2);
%msh=generateMesh(model);
```
d.	Locate the node positions that are within the area of the feet
```matlab
nodes = msh.Nodes;
elements = msh.Elements;
% LF_Node_Index = intersect(find(msh.Nodes(1,:) == LF_coords(1)),intersect(find(msh.Nodes(2,:) == LF_coords(2)),find(msh.Nodes(3,:) == LF_coords(3))));
% RF_Node_Index = intersect(find(msh.Nodes(1,:) == RF_coords(1)),intersect(find(msh.Nodes(2,:) == RF_coords(2)),find(msh.Nodes(3,:) == RF_coords(3))));
% LB_Node_Index = intersect(find(msh.Nodes(1,:) == LB_coords(1)),intersect(find(msh.Nodes(2,:) == LB_coords(2)),find(msh.Nodes(3,:) == LB_coords(3))));
% RB_Node_Index = intersect(find(msh.Nodes(1,:) == RB_coords(1)),intersect(find(msh.Nodes(2,:) == RB_coords(2)),find(msh.Nodes(3,:) == RB_coords(3))));


LF_Node_Index = 9;
RF_Node_Index = 10;
LB_Node_Index = 11;
RB_Node_Index = 12;


ii = 1;
for i = 1:1:size(msh.Elements,2)
    if sum(find (msh.Elements(:,i) == LF_Node_Index)) ~= 0
        ElemInLF(ii) = i;
        ii = ii+1;
    else
    end
end

ii = 1;
for i = 1:1:size(msh.Elements,2)
    if sum(find (msh.Elements(:,i) == RF_Node_Index)) ~= 0
        ElemInRF(ii) = i;
        ii = ii+1;
    else
    end
end

ii = 1;
for i = 1:1:size(msh.Elements,2)
    if sum(find (msh.Elements(:,i) == LB_Node_Index)) ~= 0
        ElemInLB(ii) = i;
        ii = ii+1;
    else
    end
end

ii = 1;
for i = 1:1:size(msh.Elements,2)
    if sum(find (msh.Elements(:,i) == RB_Node_Index)) ~= 0
        ElemInRB(ii) = i;
        ii = ii+1;
    else
    end
end


% adc = unique(elements(:,ElemInLF));
% nodes(:,adc(find(nodes(3,adc)==0)))

Ele_link_LF_index = elements(:,ElemInLF); %Node index of elements that contains the reference node
Ele_link_LF_index2 = Ele_link_LF_index(1:4,:); % Only include the element vertex. The others are midpoint node
adc = unique(Ele_link_LF_index2);%Get rid of the repeated element vertex.
adc(find(adc == LF_Node_Index)) = [];%Get rid of the reference node
LF_surroundingNodes_Index = adc(find(nodes(3,adc)==0));
LF_surroundingNodes_coords = nodes(:,LF_surroundingNodes_Index);%Find the node coordinates that is one element away from the reference node. This will be used to add vertex to the geometry in the next step.

Ele_link_RF_index = elements(:,ElemInRF); %Node index of elements that contains the reference node
Ele_link_RF_index2 = Ele_link_RF_index(1:4,:); % Only include the element vertex. The others are midpoint node
adc = unique(Ele_link_RF_index2);%Get rid of the repeated element vertex.
adc(find(adc == RF_Node_Index)) = [];%Get rid of the reference node
RF_surroundingNodes_Index = adc(find(nodes(3,adc)==0));
RF_surroundingNodes_coords = nodes(:,RF_surroundingNodes_Index);%Find the node coordinates that is one element away from the reference node. This will be used to add vertex to the geometry in the next step.

Ele_link_LB_index = elements(:,ElemInLB); %Node index of elements that contains the reference node
Ele_link_LB_index2 = Ele_link_LB_index(1:4,:); % Only include the element vertex. The others are midpoint node
adc = unique(Ele_link_LB_index2);%Get rid of the repeated element vertex.
adc(find(adc == LB_Node_Index)) = [];%Get rid of the reference node
LB_surroundingNodes_Index = adc(find(nodes(3,adc)==0));
LB_surroundingNodes_coords = nodes(:,LB_surroundingNodes_Index);%Find the node coordinates that is one element away from the reference node. This will be used to add vertex to the geometry in the next step.

Ele_link_RB_index = elements(:,ElemInRB); %Node index of elements that contains the reference node
Ele_link_RB_index2 = Ele_link_RB_index(1:4,:); % Only include the element vertex. The others are midpoint node
adc = unique(Ele_link_RB_index2);%Get rid of the repeated element vertex.
adc(find(adc == RB_Node_Index)) = [];%Get rid of the reference node
RB_surroundingNodes_Index = adc(find(nodes(3,adc)==0));
RB_surroundingNodes_coords = nodes(:,RB_surroundingNodes_Index);%Find the node coordinates that is one element away from the reference node. This will be used to add vertex to the geometry in the next step.
```
e.	Add the node positions as vertex to the original geometry and re-mesh the new geometry
```matlab
%% Actual model we use for analysis
model2 = createpde("structural","modal-solid");
gm2 = multicuboid(Width,Depth,L_te);
addVertex(gm2,"Coordinates",origins);
addVertex(gm2,"Coordinates",[LF_surroundingNodes_coords';RF_surroundingNodes_coords';LB_surroundingNodes_coords';RB_surroundingNodes_coords']);
for i = 1:1:size(PoI,1)
    addVertex(gm2,"Coordinates",PoI(i,:));
end


model2.Geometry = gm2;
generateMesh(model2,"Hface",{[1 2],0.05},Hmax=hmax,GeometricOrder="linear",Hgrad=1.2); %need to be kept the same as the msh method in model in the last section
structuralProperties(model2,"YoungsModulus",E, ...
    "PoissonsRatio",nu, ...
"MassDensity",rho);
```
f.	Apply vertex type multipoint-constrain to define the interface, the degrees of freedom of which will be retained after Craig-Bampton reduction
```matlab
LF_surroundingVertex_Index = [1:1:size(LF_surroundingNodes_Index,1)]+12;
RF_surroundingVertex_Index = [size(LF_surroundingNodes_Index,1)+1:1:size(RF_surroundingNodes_Index,1)+size(LF_surroundingNodes_Index,1)]+12;
LB_surroundingVertex_Index = [size(RF_surroundingNodes_Index,1)+size(LF_surroundingNodes_Index,1)+1:1:size(RF_surroundingNodes_Index,1)+size(LF_surroundingNodes_Index,1)+size(LB_surroundingNodes_Index,1)]+12;
RB_surroundingVertex_Index = [size(RF_surroundingNodes_Index,1)+size(LF_surroundingNodes_Index,1)+size(LB_surroundingNodes_Index,1)+1:1:size(RF_surroundingNodes_Index,1)+size(LF_surroundingNodes_Index,1)+size(LB_surroundingNodes_Index,1)+size(RB_surroundingNodes_Index,1)]+12;
LF_surroundingVertex_Index = [9 LF_surroundingVertex_Index];
RF_surroundingVertex_Index = [10 RF_surroundingVertex_Index];
LB_surroundingVertex_Index = [11 LB_surroundingVertex_Index];
RB_surroundingVertex_Index = [12 RB_surroundingVertex_Index];
structuralBC(model2, ...
    'Vertex',LF_surroundingVertex_Index, ...
    'Constraint','multipoint', ...
    'Reference',origins(1,:));

structuralBC(model2, ...
    'Vertex',RF_surroundingVertex_Index, ...
    'Constraint','multipoint', ...
    'Reference',origins(2,:));

structuralBC(model2, ...
    'Vertex',LB_surroundingVertex_Index, ...
    'Constraint','multipoint', ...
    'Reference',origins(3,:));

structuralBC(model2, ...
    'Vertex',RB_surroundingVertex_Index, ...
    'Constraint','multipoint', ...
    'Reference',origins(4,:));



figure(1)
pdegplot(model2,'Vertexlabels','on','FaceAlpha',0.5);

Q = meshQuality(model2.Mesh,"aspect-ratio");
elemIDs = find(Q < 0.5);
figure(2)
pdemesh(model2.Mesh,"FaceAlpha",0.5)
hold on
pdemesh(model2.Mesh.Nodes,model2.Mesh.Elements(:,elemIDs), ...
    "FaceColor","blue","EdgeColor","blue")
mv03_percent = volume(model2.Mesh,elemIDs)/volume(model2.Mesh)*100

figure(3)
hist(Q);
xlabel('Element Shape Quality', 'fontweight','b');
ylabel('Number of Elements', 'fontweight','b');

model2.SolverOptions.MaxShift = 500;

```




