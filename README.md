# CG-GMP-PBCA
`Computer Graphics`
`Geometry Modeling And Processing`
`Physics-based Computer Animation`

## Contents

### Rigid Body Dynamics Solver based on Impulse
`RigidBody` `Impulse`

[https://github.com/rainwl/CG-GMP-PBCA/issues/1#issue-1812139717](https://user-images.githubusercontent.com/51992995/254614281-93e7a466-45c4-4569-a100-1eb618431330.mp4)

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


$$ K \gets \frac{1}{M}I-(Rr_{i})^{*}I^{-1}(Rr_{i})^{*} $$

$$ K \gets \frac{1}{M}I-(Rr_{i})^{*}I^{-1}(Rr_{i})^{*} $$


`Update v and w`

$$v\gets v+\frac{1}{M}j $$

$$\omega \gets \omega +I^{-1}(Rr_{i}\times j)$$

![](https://pic4rain.oss-cn-beijing.aliyuncs.com/img/RBD_algorithm.png)


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

