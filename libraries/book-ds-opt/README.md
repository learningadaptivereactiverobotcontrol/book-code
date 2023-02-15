# ds-opt
Toolbox including optimization techniques for estimation of Globally Asymptotically Stable Dynamical Systems focused on Linear Parameter Varying formulation with GMM-based mixing function and different Lyapunov candidate functions as proposed in [1], where a non-linear DS formulated as:
<p align="center">
<img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/img/f_x.gif"></>
                                                                           
is learned from demonstrations in a decoupled manner. Where the GMM parameters <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/img/theta_gamma.gif"> used to parametrize <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/img/gamma.gif"> are estimated via the physically-consistent GMM approach proposed in [1] and provided in [phys-gmm](https://github.com/nbfigueroa/phys-gmm) and the DS parameters <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/img/DS_params.gif"> are estimated via semi-definite optimization problem that ensures global asymptotic stability of the system via constraints derived from either a:
- QLF (Quadratic Lyapunov Function): <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/img/stab_qlf.gif">
- P-QLF(Parametrized QLF):  <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/img/stab_pqlf.gif">

This allows us to accurately encode highly non-linear, non-monotic trajectories as the ones below:

<p align="center">
  <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Lshape_lpvO3.png" width="220">
<img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Ashape_lpvO3.png" width="220"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Sshape_lpvO3.png" width="220"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Ashape_lpvO3.png" width="220">
</>
  
while ensuring global asymptotic stability. 

---
  
### Dependencies
- **[Necessary]** [phys-gmm](https://github.com/nbfigueroa/phys-gmm.git):  This package needs the **physically-consisent** GMM (PC-GMM) fitting proposed in [1]. Please download it, install its dependencies and place it in your MATLAB workspace path.

- **[Optional]**: For comparison purposes, this toolbox also includes demo scripts for DS learning with SEDS [2].  

  - To run the SEDS learning demo script, download SEDS implementation from 
  ```
  $ git clone https://bitbucket.org/khansari/seds SEDS 
  ```
  and place it in the ```thirdparty/seds``` folder.

---

### Running the demo scripts
Each ```demo_learn_*.m``` script includes self-explanatory code-block instructions to learn DS with: 
- ```demo_learn_lpvds.m```: The proposed LPV-DS approach [1] allowing to test different mixing function estimation approaches and DS parameter constraint optimization variants.
- ```demo_learn_seds.m```: The se-DS approach [2] allowing to test different estimation approaches for the intial GMM and different object functions for the DS optimization.

#### Datasets
In each of these scripts you can load the datasets shown below or any motion from the ``LASA Handwriting dataset``. Also with the ```demo_drawData_DS.m``` you can draw your own 2D datasets on a GUI and 

#### Incremental Algorithm
The ``demo_incremental_lpvDS.m`` script shows an implementation of the incremental learning framework proposed in [1] using the 2D datasets used in the paper or by drawing your own 2D datasets!

---

### Example Datasets
These examples + more datasets are provided in ```
./datasets``` folder. Following we show some **notably challenging motions** that cannot be accurately encoded with SEDS [3] **(1st Column)** or a PC-GMM-based LPV-DS [1] with a simple Quadradtic Lyapunov Function (QLF) **(2nd Column)**, yet can be correctly encoded with the proposed PC-GMM-based LPV-DS with a parametrized QLF (P-QLF) [1] **(3rd Column)** yielding comparable **(in some cases BETTER)** results than the global diffeomorphic matching approach [3] **(4th Column)**, which is the state-of-the-art approach known to outperform SEDS. To reproduce result for the latter, please contact the original authors of that paper.

-  **2D S-shape Dataset**  
<p align="center">
  <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Sshape_seds.png" width="220">
<img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Sshape_lpvO1.png" width="220"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Sshape_lpvO3.png" width="220"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Sshape_diff.png" width="210">
</>

-  **2D Multi-Behavior (Single Target) Dataset**  
<p align="center">
  <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Multi_seds.png" width="215">
<img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Multi_lpv01.png" width="220"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Multi_lpv03.png" width="220"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Multi_diff.png" width="220">
</>  

-  **2D Messy Snake Dataset**  
<p align="center">
  <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Messy-snake_seds.png" width="215">
<img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Messy-snake_lpvO1.png" width="220"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Messy-snake_lpvO3.png" width="220"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Messy-snake_diff.png" width="225">
</>
  
-  **2D SharpC-shape from LASA Handwriting Dataset**  
<p align="center">
  <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/CSharp_seds.png" width="220">
<img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/CSharp_lpvO1.png" width="220"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/CSharp_lpvO3.png" width="220"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/CSharp_diff.png" width="220">
</>

-  **2D N-shape from LASA Handwriting Dataset**  
<p align="center">
  <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Nshape_seds.png" width="220">
<img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Nshape_lpvO1.png" width="225"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Nshape_lpvO3.png" width="220"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Nshape_diff.png" width="210">
</>


-  **2D Hee-shape from LASA Handwriting Dataset**  
<p align="center">
  <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Hee_seds.png" width="220">
<img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Hee_lpvO1.png" width="220"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Hee_lpvO3.png" width="220"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/Hee_diff.png" width="220">
</>
  
-  **2D Snake-shape from LASA Handwriting Dataset**  
<p align="center">
  <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/LASASnake_seds.png" width="220">
<img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/LASASnake_lpvO1.png" width="220"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/LASASnake_lpvO3.png" width="220"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/LASASnake_diff.png" width="220">
</>
  
The following 3D datasets where collected via kinesthetic teaching of a KUKA LWR 4+ robot and processed using the code provided in the [easy-kinesthetic-recording](https://github.com/nbfigueroa/easy-kinesthetic-recording) package.  

-  **3D Sink Motion for "Inspection Line" Task**  
<p align="center">
  <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/inspection-demo.gif" width="290"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-Sink_lpvO1.png" width="270"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-Sink_lpvO3.png" width="270"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-Sink_pc-gmm.png" width="270">
<img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-Sink_seds.png" width="270"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-Sink_diff.png" width="270">
</>

-  **3D Via-point Motion for "Branding Line" Task**  
<p align="center">
  <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/branding-demo.gif" width="290"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-Via-point-paper_lpvO1_good.png" width="270"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-Via-point-paper_lpvO3_good.png" width="270"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-Via-point-paper_pc-gmm.png" width="290">
<img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-Via-point-paper_seds.png" width="270"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-Via-point-paper_diff.png" width="270">
</>
  
-  **3D Cshape-top Motion for "Shelf Arranging" Task**  
<p align="center">  
  <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/tasks/Scenario3_top_demo.gif" width="290"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-CShape-top_lpvO1.png" width="250"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-CShape-top_lpvO3.png" width="270"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-CShape-top_pcgmm.png" width="280">
<img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-CShape-top_seds.png" width="270"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-CShape-top_diff.png" width="270">
</>  
  
-  **3D Cshape-botton Motion for "Shelf Arranging" Task**  
<p align="center">  
  <img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/tasks/Scenario3_bottom_demo.gif" width="290"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-CShape-bottom_lpvO1.png" width="250"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-CShape-bottom_lpvO3.png" width="270"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-CShape-bottom_pcgmm.png" width="280">
<img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-CShape-bottom_seds.png" width="270"><img src="https://github.com/nbfigueroa/ds-opt/blob/master/figs/3D-Datasets/3D-CShape-bottom_diff.png" width="270">
</>  
  
**References**     
> [1] Figueroa, N. and Billard, A. (2018) A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning. In Proceedings of the 2nd Conference on Robot Learning (CoRL). Accepted.     
> [2] Khansari Zadeh, S. M. and Billard, A. (2011) Learning Stable Non-Linear Dynamical Systems with Gaussian Mixture Models. IEEE Transaction on Robotics, vol. 27, num 5, p. 943-957.    
> [3] N. Perrin and P. Schlehuber-Caissier, “Fast diffeomorphic matching to learn globally asymptotically stable nonlinear dynamical systems,” Systems & Control Letters (2016).

**Contact**: [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387) (nadia.figueroafernandez AT epfl dot ch)

**Acknowledgments**
This work was supported by the EU project [Cogimon](https://cogimon.eu/cognitive-interaction-motion-cogimon) H2020-ICT-23-2014.
