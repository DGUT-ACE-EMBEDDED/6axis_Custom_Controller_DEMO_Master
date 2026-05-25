clear;
clc;
close all;

deta_a = -0.055;
%%deta_d的计算方式还没弄出来，一般忽略
deta_d = 0;


%%定义link2，3
l2 = 0.380;
l3 = 0.3783;

L1 = Link([0, 0,  0,      0], 'revolute', 'modified');
L2 = Link([0, 0,     0,  -pi/2], 'revolute', 'modified');
L3 = Link([0,deta_d, l2,   0], 'revolute', 'modified');
L4 = Link([0, l3,deta_a,-pi/2], 'revolute', 'modified');
L5 = Link([0, 0,0,pi/2], 'revolute', 'modified');
L6 = Link([0, 0,     0, -pi/2], 'revolute', 'modified');

robot = SerialLink([L1 L2 L3 L4 L5 L6]);

%%目标位置(后面需要改成陀螺仪坐标的四元数)，rpy的是自身zyx顺序的旋转
x = 0.2;
y = 0.0;
z = 0.3;
r = 0;
p = 0;
yaw = 0;

%%定义末端与腕部相对位置，求 6->0部分的数值 
d_end = 0.0;
Rend26 = [cos(pi/2),0,sin(pi/2);
          0        ,1,0;
          -sin(pi/2),0,cos(pi/2)]*[cos(pi),-sin(pi),0;
                                   sin(pi),cos(pi),0;
                                   0,0,1];
Tend26 = [[Rend26;0,0,0],[0;0;d_end;1]]
Tend26\eye(size(Tend26))
%%这里要改成陀螺仪的四元数解算,所以要输入四元数
Rimu =  zyx_quaternion2R(r,p,yaw);

Timu = [[Rimu;0,0,0],[x;y;z;1]]



T60 = getT60(Tend26,Timu)

 %%求前三轴j123
j123 = upTranslation(l2,l3,T60(1,4),T60(2,4),T60(3,4),deta_a)
%%求4->0的齐次变换矩阵
T10 = getTr(0,j123(1),0,0,0);
T21 = getTr(-pi/2,j123(2),0,0,0);
T32 = getTr(0,j123(3),l2,0,deta_d);

T43 = getTr(-pi/2,0,deta_a,l3,0);

%%test
%q = [0,-1.9796,-0.1425,0,0,0]
%robot.A(1,q)*robot.A(2,q)*robot.A(3,q)*robot.A(4,q)

T40 = T10*T21*T32*T43  

%%求后三轴旋转，先提取其旋转矩阵
R60 = T60(1:3,1:3)
R40 = T40(1:3,1:3)
%%由于旋转矩阵与欧拉角无关，如上面的准备， 我们可以求出6->4 的旋转，类似的有
%%R60 =  R40 * R64,   R64 = R40_1 *  R60 ,由于旋转矩阵是正交的 ，那么有   R64  = R40^T * R60
R64 = R40'* R60
%%求zyz
beta_2 = atan2(sqrt(R64(3,1)^2 + R64(3,2)^2),R64(3,3))
eps_val = 1e-6;
if abs(beta_2) < eps_val
    alpha_1 = 0
    gama_3 = atan2(-R64(1,2),R64(1,1))
elseif abs(beta_2 - pi) < eps_val 
    alpha_1 = 0;
    gama_3 = atan2(R64(1,2),-R64(1,1))
else 
    alpha_1 = atan2(R64(2,3),R64(1,3))
    gama_3 = atan2(R64(3,2),-R64(3,1))
end

beta_2 = -beta_2