# GAMES 01 

## Why Study Computer Graphics
- fundamental intellectual challenges
  - creates and iteracts with realistic virtual world
  - requires understanding of all aspects of physical world
  - new computing methods displays technologies
- Technical Challenges
  - Math of (perspective) projections ,curves,surfaces
  - physics of lighting and shading
  - representing/operating shapes in 3D 
  - Animation / simulation
  - 3D graphics software programming and hardware

## Course Topics
- Rasterization 光栅化(OpenGL,shader如何运作的)
- Curves and Meshes 曲线曲面(几何)
- Ray Tracing 光线追踪(动画,电影,路径追踪,表面建模,光线传播方法)
- Animation/SImulation 


## Rasterization
- Project geometry primitives (3D triangles/polygons) onto the screen
- break projected primitives into fragments(pixels)
- Gold standard in video games

calculate intersection and shading

real time:30 fps

offline

catmull-clark subdivision


treat off (为了达成某些目标,牺牲其他目标)

mass-spring system质量弹簧系统

CV:一切需要猜测的地方.computer vision
DL:deep learning topics

涉及到理解,猜测,异议的不是图形学

图形学:model 渲染成 image

视觉:从image 认出 model

academic integrity学术诚信

swift and brutal迅捷而残暴

linear algebra线性代数

## 101.2 Linear Algebra

announcements公告,注意事项

slides and recordings of lecture 1 now available 第一讲的幻灯片和录音已经提供了


Graphics's dependencies
- Basic mathematics
  - linear algebra,calculus,statistics线代,微积分,统计
- Basic Physics
  - optics,mechanics光学力学
  - (假设当光不能直线传播的时候,以光波的形式进行传递,与表面如何作用)
- Misc
  - Signal processing信号处理(走样,反走样)
  - Numerical analysis 数值分析(渲染就是解一个递归问题的积分)
  - 仿真,解有限元,扩散方程等问题
- A bit of aesthetics 美学

- More dependent on linear algebra
  - vectors(dot products,cross)
  - Matrices(matrix-matrix,matrix-vector)

Vector Normalization 
magnitude of a vector(数量)
unit vector

Geometrically:Parallelogram law & Triangle law几何:平行四边形定律和三角形定律

Algebraically:Simply add coordiantes代数:简单地添加坐标

Cartesian Coordinates笛卡儿坐标
orthogonal unit非正交单胞模型 互相垂直的
图形学默认的向量,是一列数,从上往下写
A=(),括号里是竖着的x y
AT,转置,行列互换
算向量长度比较简单(用垂直单位坐标系) 


Vector Multiplication

Dot product (scalar )
Cross product
Orthonormal bases and coordinate frames

