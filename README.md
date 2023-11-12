# Computer Graphics

CG-GMP-PBCA

`Computer Graphics`
`Geometry Modeling And Processing`
`Physics-based Computer Animation`

## Contents

### Rigid Body Dynamics Solver based on Impulse
`RigidBody` `Dynamics` `Impulse`

![https://github.com/rainwl/CG-GMP-PBCA/issues/12#issue-1837572878](https://user-images.githubusercontent.com/51992995/258563389-cfa6657d-8476-46c9-93ec-f1d80a01fed3.gif)

The goal of simulation is to update the state variable **s**<sup>k</sup> over time.
And we use `semi-implicit(leapfrog)` approach.

---

$$v^{[0.5]}=v^{[-0.5]}+\Delta tM^{-1}f^{[0]}$$

$$x^{[1]}=x^{[0]}+\Delta tv^{[0.5]}$$

`Rigid Body Simulation`

$$f_{i}\gets Force(x_{i},v_{i})$$

$$f\gets \sum f_{i}$$

$$v\gets v+\Delta tM^{-1}f$$

$$x\gets x+\Delta tv$$

$$R\gets Matrix.Rotate(q)$$

$$\tau_{i}\gets (Rr_{i})\times f_{i}$$

$$\tau \gets \sum \tau _{i}$$

$$I\gets RI_{ref}R^{T}$$

$$\omega \gets \omega +\Delta t(I)^{-1}\tau $$

$$q\gets q+\begin{bmatrix}0  &\frac{\Delta t}{2} \omega \end{bmatrix}\times q$$



For every vertex 

$$x_{i}  \gets   x +Rr_{i} $$

if $\phi (x_{i})<0$

$$v_{i}\gets v+\omega \times Rr_{i}$$ 

if $v_{i}\cdot N<0$

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

### Shape Matching

![https://github.com/rainwl/CG-GMP-PBCA/issues/17#issue-1854939793](https://user-images.githubusercontent.com/51992995/261312764-10d16aa5-800c-4e95-b5cc-1fe3e0caaa59.gif)

First,we move vertices independently by its velocity,with collision and friction being handled.

Second,enforce the rigidity constraint to become a rigid body again.

---

$$\{ c,R \} =argmin\sum_{i}^{}  \frac{1}{2}\begin{Vmatrix}c+Rr_{i}-y_{i}\end{Vmatrix}^{2} $$

$$\{ c,A \} =argmin \sum_{i}^{} \frac{1}{2}\begin{Vmatrix}c+Rr_{i}-y_{i}\end{Vmatrix}^{2}$$

$$\frac{\partial E}{\partial c} = \sum_{i}^{}c+Ar_{i}-y_{i}= \sum_{i}^{}c-y_{i}=0$$

$$c = \frac{1}{N} \sum_{i}^{} y_{i}$$

$$\frac{\partial E}{\partial A} = \sum_{i}^{}(c+Ar_{i}-y_{i})r_{i}^{T} = 0$$

$$A=( \sum_{i}^{}(y_{i}-c)r_{i}^{T})(\sum_{i}^{}r_{i}r_{i}^{T})^{-1}$$

$$A=RS$$

`R` is `rotation` and `S` is `deformation`

$$A=UDV^{T}$$

$$A=(UV^{T})(VDV^{T})$$

`local deformation` VDV<sup>T</sup> = S

$$R=Polar(A)$$

The polar decomposition is `unique`

For every vertex

$$v_{i}\gets (c+Rr_{i}-x_{i})/\Delta t$$

$$x_{i}\gets c+Rr_{i}$$

### Position-Based Dynamics
`PBD` `Cloth`

![https://github.com/rainwl/CG-GMP-PBCA/issues/10#issue-1837554865](https://user-images.githubusercontent.com/51992995/258560892-91bf53f2-1130-48eb-82af-83646a1ca358.gif)

---

`constraint`

$$\phi (x) = \begin{Vmatrix}x_{i}-x_{j}\end{Vmatrix}-L=0$$

$$\{x_{i}^{new},x_{j}^{new}\}=argmin\frac{1}{2} \{m_{i}\begin{Vmatrix}x_{i}^{new}-x_{i}\end{Vmatrix}^{2}+m_{j}\begin{Vmatrix}x_{j}^{new}-x_{j}\end{Vmatrix}^{2} \}$$

$$x_{i}^{new}\gets x_{i}-\frac{m_{j}}{m_{i}+m_{j}}(\begin{Vmatrix}x_{i}-x_{j}\end{Vmatrix}-L) \frac{x_{i}-x_{j}}{\begin{Vmatrix}x_{i}-x_{j}\end{Vmatrix}} $$

`Gauss-Seidel Approach` `stochastic gradient descent`

For k = 0...K

For every edge e = {i,j}

$$x_{i}\gets x_{i}-\frac{1}{2} (\begin{Vmatrix}x_{i}-x_{j}\end{Vmatrix}-L_{e}) \frac{x_{i}-x_{j}}{\begin{Vmatrix}x_{i}-x_{j}\end{Vmatrix}} $$

$$x_{j}\gets x_{j}+\frac{1}{2} (\begin{Vmatrix}x_{i}-x_{j}\end{Vmatrix}-L_{e}) \frac{x_{i}-x_{j}}{\begin{Vmatrix}x_{i}-x_{j}\end{Vmatrix}} $$

`Jacobi Approach`

For k = 0...K

For every vertex i

$$x_{i}^{new}\gets 0$$

$$n_{i}\gets 0$$

For every edge e = {i,j}

$$x_{i}^{new}\gets x_{i}^{new}+x_{i}-\frac{1}{2} (\begin{Vmatrix}x_{i}-x_{j}\end{Vmatrix}-L_{e}) \frac{x_{i}-x_{j}}{\begin{Vmatrix}x_{i}-x_{j}\end{Vmatrix}}$$

$$x_{j}^{new}\gets x_{j}^{new}+x_{j}-\frac{1}{2} (\begin{Vmatrix}x_{i}-x_{j}\end{Vmatrix}-L_{e}) \frac{x_{i}-x_{j}}{\begin{Vmatrix}x_{i}-x_{j}\end{Vmatrix}}$$

$$n_{i}\gets n_{i}+1$$

$$n_{j}\gets n_{j}+1$$

For every vertex i

$$x_{i}\gets (x_{i}^{new}=\alpha x_{i})/(n_{i}+\alpha )$$

---

#### `PBD`

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

`effect`

- The number of iterations
- The mesh resolution

---

`Strain Limiting`

Do simulation,update x and v

$$v\gets ...$$

$$x\gets ...$$

Strain limiting starts

$$x^{new}\gets Projection(x)$$

$$v\gets v+(x^{new}-x)/\Delta t$$

$$x\gets x^{new}$$

### Projective Dynamics

### Constrained Dynamics


### Implicit Integration Cloth Solver

![https://github.com/rainwl/CG-GMP-PBCA/issues/11#issue-1837564243](https://user-images.githubusercontent.com/51992995/258561289-31c6ed76-81ed-429a-9f39-c3e948eaceca.gif)

---

`gradient`

$$g=\frac{1}{\Delta t^{2}}M(x-\tilde{x}) -f(x)$$

`loop through each edge`

$$g_{i}\gets g_{i}+k(1-\frac{L_{e}}{\left \| x_{i}-x_{j} \right \| } )(x_{i}-x_{j}),g_{j}\gets g_{j}+k(1-\frac{L_{e}}{\left \| x_{i}-x_{j} \right \| } )$$

`update`

$$x_{i}\gets x_{i}-(\frac{1}{\Delta t^{2}}m_{i}+4k )^{-1}g_{i}$$

`colliding and apply impulse`

$$v_{i}\gets v_{i}+\frac{1}{\Delta t}(c+r\frac{x_{i}-c}{\left \|x_{i}-c  \right \| }-x_{i} )$$

$$x_{i}\gets c+r\frac{x_{i}-c}{\left \|x_{i}-c  \right \| } $$


---

#### Mass-Spring System(explicit integration)

Explicit integration suffers from `numerical instability` caused by `overshooting`,when the
`stiffness` k and/or the `time step` dt is too large.

Edge list:`E`

Edge length list:`L`

`Compute Spring Forces`

For every edge e

$$i\gets E[e][0]$$

$$j\gets E[e][1]$$

$$L_{e}\gets L[e]$$

$$f\gets -k(\begin{Vmatrix}x_{i}-x_{j}\end{Vmatrix}-L_{e})\frac{x_{i}-x_{j}}{\begin{Vmatrix}x_{i}-x_{j}\end{Vmatrix}} $$

$$f_{i}\gets f_{i}+f$$

$$f_{j}\gets f_{j}-f$$

`A Particle System`

For every vertex

$$f_{i}\gets Force(x_{i},v_{i})$$

$$v_{i}\gets v_{i}+\Delta tm_{i}^{-1}f_{i}$$

$$x_{i}\gets x_{i}+\Delta tv_{i}$$

---

#### Mass-Spring System(implicit integration)

Implicit integration is a better solution to `numerical instability`.The idea is to integrate both `x` and `v` implicitly.

We should note that it is implicit integration that can `overcome` overshooting,overshooting is overcome by error attenuation.

And `solving equations` is a special case of `optimization problem`.

`traditional`

$$v^{[1]}=v^{[0]}+\Delta tM^{-1}f^{[1]}$$

$$x^{[1]}=x^{[0]}+\Delta tv^{[1]}$$

`or`

$$x^{[1]}=x^{[0]}+\Delta tv^{[0]}+\Delta t^{2}M^{-1}f^{[1]}$$

$$v^{[1]}=(x^{[1]}-x^{[0]})/\Delta t$$


Assuming that `f` is *holonomic* (only depends on position).depending on `x` only,our question is how to solve:

$$x^{[1]}=x^{[0]}+\Delta tv^{[0]}+\Delta t^{2}M^{-1}f(x^{[1]})$$

They are equivalent:

$$x^{[1]}=argminF(x)$$

`for`

$$F(x)=\frac{1}{2\Delta t^{2}} \begin{Vmatrix}x-x^{[0]}-\Delta tv^{[0]}\end{Vmatrix}_{M}^{2}+E(x)$$

$$\begin{Vmatrix}x\end{Vmatrix}^{2}_{M}=x^{T}Mx$$

`because`

$$\nabla F(x^{[1]})=\frac{1}{\Delta t^{2}}M(x^{[1]}-x^{[0]}-\Delta tv^{[0]})-f(x^{[1]})=0 $$

`equivalent`

$$x^{[1]}-x^{[0]}-\Delta tv^{[0]}-\Delta t^{2}M^{-1}f(x^{[1]})=0$$

`Note` that this is applicable to every system,not just a mass-spring system.


---

#### Newton-Raphson Method

`optimization problem`

$$x^{[1]}=argminF(x)$$

F(x) is `Lipschitz continuous`

`goal`

$$0=F'(x)\approx F'(x^{(k)})+F''(x^{(k)})(x-x^{(k)})$$

`Newton's method`

Initialize x<sup>(0)</sup>

For k = 0...K

$$\Delta x \gets -(F''(x^{(k)}))^{-1}F'(x^{(k)})$$

$$x^{(k+1)}\gets x^{(k)}+\Delta x$$

$$x^{[1]}\gets x^{(k+1)}$$

`because`

$$F''(x^{(k)})=\frac{\partial F^{2}(x^{(k)})}{\partial x^{2}} =\frac{1}{\Delta t^{2}} M+H(x^{(k)})$$

`so`

the goal is to solve

$$\mathbf{A} \Delta \mathbf{x} =\mathbf{b} $$


---

`Jocobi`

$$\Delta x\gets 0$$

For k= 0...K

`r` residual error

$$r\gets b-A\Delta x$$

if

`convergence condition`

$$\begin{Vmatrix}r\end{Vmatrix}<\varepsilon $$

break

`diagnonal of A` D

$$\Delta x\gets \Delta x+\alpha D^{-1}r$$

---
`Chebyshev Accleration`


---

### Finite Element Method
`St.Venant-Kirchhoff(StVk)` `explicit time integration` `tetrahedral`

![https://github.com/rainwl/CG-GMP-PBCA/issues/9#issue-1837076168](https://user-images.githubusercontent.com/51992995/258483475-240e3026-d414-4937-a532-a434bd07317a.gif)


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

### Two-way Coupling Shallow Wave

![https://github.com/rainwl/CG-GMP-PBCA/issues/13#issue-1837585974](https://user-images.githubusercontent.com/51992995/258565200-a8c4c031-15ae-4ffa-a589-f34cd95bcffd.gif)

---

`Height Field`

$$\frac{du(x)}{dt}=-\frac{1}{\rho }  \frac{dP(x)}{dx} $$



`shallow wave equation`

$$\frac{d^{2}h }{dt^{2}}=\frac{h}{\rho }  \frac{d^{2}p}{dx^{2}} $$


`Volume Preservation`

$$\sum h_{i}(t+\Delta t)=V+\frac{\Delta t^{2}H}{\Delta x^{2}\rho } \sum (P_{i-1}-P_{i})(P_{i+1}-P_{i})$$

`One-way Coupling`

$$h_{i,j}^{new}\gets  h_{i,j}^{new}+(v_{i-1,j}+v_{i+1,j}+v_{i,j-1}+v_{i,j+1}-4v_{ij})\ast rate$$
