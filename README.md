# CG-GMP-PBCA
`Computer Graphics`
`Geometry Modeling And Processing`
`Physics-based Computer Animation`

## Contents

### Rigid Body Dynamics Solver based on Impulse
`RigidBody` `Dynamics` `Impulse`

[https://github.com/rainwl/CG-GMP-PBCA/issues/6#issue-1832234015](https://user-images.githubusercontent.com/51992995/257672696-50282f6e-5fe3-4ae3-8dc2-1909cf375361.gif)

[https://github.com/rainwl/CG-GMP-PBCA/issues/3#issue-1829466736](https://user-images.githubusercontent.com/51992995/257269338-6fc142b8-b077-4721-8e05-a8e5701f8f5f.mp4)


For every vertex $x_{i}  \gets   x +Rr_{i} $ 

if $\phi (x_{i})<0$

$v_{i}\gets v+\omega \times Rr_{i}$ , if $v_{i}\cdot N<0$

$$v_{N,i}\gets ( v_{i} \cdot N)N$$

$$v_{T,i}\gets v_{i}-v_{N,i}$$

$$a\gets max(1-\mu T(1+ \mu_{N})\left \| V_{N,i} \right \| / \left \| V_{T,i} \right \| ,0)$$

$$v_{N,i}^{new} \gets - \mu_{N} V_{N,i}$$

$$v_{T,i}^{new} \gets av_{T,i}$$

$$v_{i}^{new} \gets v_{N,i}^{new}+v_{T,i}^{new}$$

`compute the impluse j`


$$ K \gets \frac{1}{M}I-(Rr_{i})^{\ast }I^{-1}(Rr_{i})^{\ast } $$

$$ j\gets K^{-1}(v_{i}^{new} -v_{i}) $$


`Update v and w`

$$v\gets v+\frac{1}{M}j $$

$$\omega \gets \omega +I^{-1}(Rr_{i}\times j)$$

### Position-Based Dynamics
`PBD` `Cloth`

[https://github.com/rainwl/CG-GMP-PBCA/issues/4#issue-1829485434](https://user-images.githubusercontent.com/51992995/257270994-6ce00eb6-43e7-49ad-af84-e6f59dfa214a.mp4)

For k = 0...K

For every vertex i

$$x_{i}^{new}\gets  \vec{0} $$

$$n_{i}\gets  \vec{0} $$

For every edge e = {i,j}

$$x_{i}^{new} \gets x_{i}^{new} + x_{i}-\frac{1}{2} (\left \| x_{i}-x_{j} \right \| -L_{e})\frac{x_{i}-x_{j}}{\left \| x_{i}-x_{j} \right \| } $$

$$x_{j}^{new} \gets x_{j}^{new} + x_{j}-\frac{1}{2} (\left \| x_{i}-x_{j} \right \| -L_{e})\frac{x_{i}-x_{j}}{\left \| x_{i}-x_{j} \right \| } $$

$$n_{i}\gets n_{i}+1$$

$$n_{j}\gets n_{j}+1$$

For every vertex i

$$x_{i}\gets (x_{i}^{new}+\alpha x_{i} )/(n_{i}+\alpha )$$

### Implicit Integration Cloth Solver

$v^{[1]} = v^{[0]} +\Delta  t \mathbf{M} ^{-1}f^{[1]}$

### Finite Element Method
`St.Venant-Kirchhoff(StVk)` `explicit time integration` `tetrahedral`

[https://github.com/rainwl/CG-GMP-PBCA/issues/2#issue-1829312805](https://user-images.githubusercontent.com/51992995/257252627-176caa0e-24a0-4d70-b230-ce17a0c01aee.mp4)

First simulate the object as a simple particle system,each vertex has its own
x and v,and the v is under the influence of gravity.

Then we calculate the edge matrix of a tetrahedron.And based on the FEM,
calculate the `deformation gradient` F and the `Green Strain`G,and the 
`Second Piola-Kirchhoff stress`S,and finally the forces of 4 vertices.

Finally,implement the `Laplacian smoothing` over the vertex velocities.

---
The Linear FEM assumes that the deformation of any triangle is uniform inside.

For any point X in the reference,its deformed correspondence is :

$$x = FX+c$$

*`deformation gradient`*

$$F = \partial x/\partial X$$ 

*`Green Strain`*

$$G=\frac{1}{2} (F^{T} F-I)=\frac{1}{2} (VD^{2}V^{T}-I)=\begin{bmatrix} \varepsilon _{uu}   & \varepsilon _{uv}  & \varepsilon _{vu}   & \varepsilon _{vv} \end{bmatrix}$$  

*`Total energy`*

$$E = \int W(G)dA = A^{ref} W(\varepsilon _{uu} ,\varepsilon _{vv},\varepsilon _{uv})$$

*`Saint Venant-Kirchhoff Model`*

$$W(\varepsilon _{uu} ,\varepsilon _{vv},\varepsilon _{uv})=\frac{\lambda }{2} (\varepsilon _{uu}+\varepsilon _{vv})^{2}+\mu (\varepsilon _{uu}+\varepsilon _{vv}+\varepsilon _{uv})^{2} $$

*`Second Piola-Kirchhoff stress tensor`*

$$2\mu G+\lambda trace(G)I =S$$


*`[f1 f2]`*

$$\begin{bmatrix}
f1  &f2
\end{bmatrix}=-A^{ref} FS\begin{bmatrix}
X_{10}  &X_{20}
\end{bmatrix}^{-T} $$

*`Force`*

$$f_{0} = -\frac{\sigma }{6} (x_{10}\times x_{20}+ x_{20}\times x_{30}+x_{30}\times x_{10} )$$

