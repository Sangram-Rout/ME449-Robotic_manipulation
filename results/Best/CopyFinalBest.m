%Here we define the values provided in the mordern Robotics WIki pAge
r=.0475;
l=0.47/2;
w=0.3/2;
H0 = (1/r)*[[-l-w, 1, -1];[l+w, 1, 1];[l+w, 1, -1];[-l-w, 1, 1]];
F = pinv(H0);%The psedoinverse of the H0 matrix
F6 = [[0,0,0,0];[0,0,0,0];F;[0,0,0,0]];
M0e=[1,0,0,0.033;0,1,0,0;0,0,1,0.6546;0,0,0,1];%position of the end effector with respect to base
Blist = [[0;  0; 1;         0; 0.033;   0], ...%screw list
         [0; -1; 0;   -0.5076;     0;   0], ...
         [0; -1; 0;   -0.3526;     0;   0], ...
         [0; -1; 0;   -0.2176;     0;   0], ...
         [0;  0; 1;         0;     0;   0]];

A=[0,0,1,0;0,1,0,0;-1,0,0,0.50;0,0,0,1];%Starting position for the end effector
B=[0,0,1,1;0,1,0,0;-1,0,0,0.200;0,0,0,1];%second position of hovering for the end effector
B0=[0,0,1,1;0,1,0,0;-1,0,0,0.02500;0,0,0,1];%The position for grasp of end effector
C=[0,1,0,0;0,0,-1,-1;-1,0,0,0.200;0,0,0,1];%final position of hovering for the end effector
C0=[0,1,0,0;0,0,-1,-1;-1,0,0,0.0250;0,0,0,1];%final position for grasp of end effector
Kp = 8;%gain values
Ki = 2;
N=400;%no. for the trajectory generation
dt = 0.01;%time ste
[CSV] =   TrajectoryGeneratorS2copy(A,B,B0,C,C0,N);%output of traj egen
gslist = CSV(:,13);%The end effector grasping
MaxAng = 15;%angle of joints
% Guess
%                                  
Random = [pi/6,-0.8,0,-1.5,-2.2,0,0,0,0,0,0,0]; %Chassis 
     %Joint 
        %Wheel
%The configuration of the frame {b} of the mobile base, relative to the frame {s} on the floor
Tsb = [cos(Random(1)), -sin(Random(1)), 0,Random(2);sin(Random(1)),cos(Random(1)), 0,Random(3);0,0, 1,0.0963;0, 0, 0,1];
%theta = Random(4:8)';
Tb0=[1,0,0,0.1662;0,1,0,0;0,0,1,0.0026;0,0,0,1];%Reference bwtween the chassis center and the base
T0e = FKinBody(M0e,Blist,Random(4:8)');
X = Tsb*Tb0*T0e;
%saving the results
position=[];
% Creating zero actual configuration matrix and first row
%Position = zeros(Nofline,13);
%Position(1,1:12) = Random;
Position= [Random];
XerrMatrix = [];
VMatrix = [];
for i = 1 : length(CSV)-1
    %Helper for the Error function caluclator
         Xd =[[CSV(i,1:3),CSV(i,10)];
          [CSV(i,4:6),CSV(i,11)];
          [CSV(i,7:9),CSV(i,12)];
          [0, 0, 0,1]];
    Xd_nt = [[CSV(i+1,1:3),CSV(i+1,10)];
             [CSV(i+1,4:6),CSV(i+1,11)];
             [CSV(i+1,7:9),CSV(i+1,12)];
             [        0, 0, 0,             1]];
%To find the Jacobian from initial guess    
    T_0e_J = FKinBody(M0e,Blist,Random(4:8)');
    T_eb_J = inv(T_0e_J)*inv(Tb0);
    J_base = Adjoint(T_eb_J)*F6;
    thetalist = Random(4:8)';
    J_Arm = JacobianBody(Blist, thetalist);
    Je = [J_base J_Arm]

    %Feedback control lopp used to find wheelVel and Jointvel
    [V,Adxixd_Vd,Vd,Xerr,Incre_err] = FeedbackControlDaw(X,Xd,Xd_nt,Kp,Ki,dt);

    % Page473
     T_U_Thd = pinv(Je)*V;
     UThd = T_U_Thd';   
    %Seperating into Joint and wheel velocities
    Vels = [UThd(5:9), UThd(1:4)] ;
    %Output from the Next state which provides us with joint vel 
    State = NextStateDaw(Random,Vels,dt,MaxAng);
    Random=State;
    Position(i+1,:) = Random;
    Tsb_actual = [[cos(Random(1)), -sin(Random(1)), 0, Random(2)];
              [sin(Random(1)),  cos(Random(1)), 0, Random(3)];
              [0,0, 1,0.0963];[0,0, 0,1]];
    T0e_ac = FKinBody(M0e,Blist,(Random(1,4:8))');
    X = Tsb_actual*Tb0*T0e_ac;
    VMatrix(i,:) = transpose(V);
    XerrMatrix(i,:) = transpose(Xerr);
    
end
 en=[zeros(800,1); ones(1200,1);zeros(400,1)];
Position(:,13) = en;

csvwrite('Error.csv', XerrMatrix); 
csvwrite('YouBot.csv', Position);
    