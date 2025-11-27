## Example 1: Rocking Column with Corner Feet

The first example models a **solid marble column** with four rigid corner feet standing on a high-grip, stiff support medium. The height of the corner feet is negligible.
This example also covers the detailed workflow, and the contents that are *not* covered by the **[Simscape Example](https://uk.mathworks.com/help/sm/ug/model-excavator-dipper-arm.html)** start with ⭐ in this document.

Figure below shows the geometry of the column  
*(⚠️insert the figure)*

---

## Step 1: Craig-Bampton Reduction in MATLAB

### I — Define structure parameters and common properties  
```matlab
% Mass in kg.
% Length in m. 
% Time in s. 

clear all
clc

NumInterFace = 4;                                           % Number of contact feet
StiModi = 1;

g = 9.80665;                                                % Gravitational acceleration
Width = 1;                                                  % Cross section width.
Depth = 1*1.05;                                             % Cross section depth.
E = 5.42538e09;                                             % Material Young's modulus in Pa.
rho = 2256.86;                                              % Material density.
nu = 0.3;                                                   % Poisson's ratio.
L = 10;                                                     % Column length.
A = Width*Depth;
ms = rho*L*A;
L_te =L;

% Global properties

distanceToEdge = 0.2;
I = ms*(Width^2+L^2)/12;
hcg = L/2;
rcg = sqrt((hcg^2) + (Width/2*(1-distanceToEdge))^2);
p = sqrt(ms*g*rcg/I);
maS = ms;                                                   % Total mass of the system.
alphacg = atan((Width/2*(1-distanceToEdge))/hcg);

% Stiffness and damping for one horizontal frictional element.

muf = 0.8;                                                  % Friction coefficient
fh = 60;                                                    % Horizontal frequency (see the paper), used to define the frictional spring stiffness.
wN=2*pi*fh;
ksin=0.5;                                                   % Frictional element damping ratio.

k_n=(maS)*wN^2/NumInterFace/StiModi;                        % Stiffnes for one frictional spring.
c_n=ksin*2*sqrt(k_n*NumInterFace*(maS))/NumInterFace;       % Damping coefficient for one frictional damper.

% Stiffness and damping for one vertical support element.

fv = 60;                                                    % Vertical frequency (see the paper), used to define the vertical spring stiffness.
wn = 2*pi*fv;
dR = 0.5;                                                   % Vertical spring damping ratio.
Ksupport_total = (wn^2)*(maS)/StiModi;
Csupport_total = dR*2*sqrt(Ksupport_total*(maS));
kk2=Ksupport_total/NumInterFace;                            % Stiffnes for one vertical spring.
cc1=Csupport_total/NumInterFace;                            % Damping coefficient for one vertical damper.

%Initial conditions

hc = 0;
zc = 1*(hc/2 - (maS)*g/(Ksupport_total));                   % Initial deformation of one vertical spring.
```
---

### II — Generate or import the geometry  
Details can be found in the **[Simscape Example](https://uk.mathworks.com/help/sm/ug/model-excavator-dipper-arm.html)**.

(I will insert matlab code here, do not change this line GPT)

---

### III — Mesh the component  
This step determines the position of the interface nodes.

Workflow:
- Mesh once to identify the nearby vertex locations  
- Add the necessary vertices  
- Re-mesh  
- Apply vertex-type multipoint constraints  

The multipoint constraint is required because tetrahedral element nodes have only **three DOFs**, while an interface in Simulink must have **six DOFs**. The multipoint constraint forms a small rigid plane that provides 6 DOFs at the interface.

(I will insert matlab code here, do not change this line GPT)

---

### ⭐IV — Locate nodes within the area of the feet  
(I will insert matlab code here, do not change this line GPT)

---

### ⭐V — Add the node positions as geometry vertices and re-mesh 
(I will insert matlab code here, do not change this line GPT)

---

### ⭐VI — Apply vertex-type multipoint constraints for the interface DOFs  
These DOFs are retained during Craig-Bampton reduction.

(I will insert matlab code here, do not change this line GPT)

---

### VII — Check the fixed-interface modes  
Take note of the frequencies corresponding to the modes you want to keep. To know what modes to keep, you need to run a convergence analysis with different modes until the results does not change with more modes. 

⭐Optionally, run the modal analysis within a **live script** and use the **[visualize PDE results task](https://uk.mathworks.com/help/pde/ug/visualizepderesults.html)** to visualize the mode shapes.

(I will insert matlab code here, do not change this line GPT)

*(⚠️insert the modeshape figure here)*

---

### VIII — Apply Craig-Bampton reduction  
⭐List the frequency ranges corresponding to the modes you want to retain.

(I will insert matlab code here, do not change this line GPT)

---

### IX — Manually calculate the reduced-order damping matrix  
This allows assignment of different modal damping ratios to different modes.

Optional:  
Simulink’s reduced-order flexible solid can automatically compute:

- uniform modal damping  
- Rayleigh damping  

(I will insert matlab code here, do not change this line GPT)

---
