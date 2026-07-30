## Example 3: The rocking response of the Venus de Milo statue 

The third example models the **Venus de Milo** statue with a rigid pedestal glued to its base. The pedestal has four corner feet with negligible length standing on a marble support medium.

**Figure 1** shows the statue and its numerical model. 

<p align="center">
<img src="images/VenusGeo.JPG" width="500">
</p>

*Figure 1: (a) The Venus de Milo statue and (b) its corresponding full-order model.*


---

## 🚩 Before all
Replace the original craigBamptonImpl.m with the [modified version](./craigBamptonImpl.m). You should find the file at path like D:\MATLAB\R2023a\toolbox\pde\+pde\@StructuralModel. If you want to keep the original craigBamptonImpl.m, rename it to something else then put the [modified version](./craigBamptonImpl.m) to the same location. 

This modified Craig-Bampton implementation allows you to retain any desired fixed-interface modes, whereas the original implementation only allows to retain modes within one specific frequency range.

---




filename4 = ['Column-Stress-26Modes-' num2str(freqRatio) '-' num2str(AmpRatio) '.mat']; 
save(filename4,'time','StressZZ','NodePosition','RTrom');
```


