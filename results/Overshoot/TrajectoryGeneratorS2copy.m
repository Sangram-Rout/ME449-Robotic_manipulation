function [CSV] =   TrajectoryGeneratorS2copy(A,B,B0,C,C0,N)

% A=[1,0,0,0;
% 0,1,0,0;
% 0,0,1,0.500000000000000;
% 0,0,0,1];
% 
% B=[0,0,1,1;
% 0,1,0,0;
% -1,0,0,0.200000000000000;
% 0,0,0,1];
% 
% B0=[0,0,1,1;
% 0,1,0,0;
% -1,0,0,0.0250000000000000;
% 0,0,0,1];
% 
% C=[0,1,0,0;
% 0,0,-1,-1;
% -1,0,0,0.200000000000000;
% 0,0,0,1];
% C0=[0,1,0,0;
% 0,0,-1,-1;
% -1,0,0,0.0250000000000000;
% 0,0,0,1];

Tf = 1;
%N = 100;
method = 3;
traj = ScrewTrajectory(A, B, Tf, N, method);
traj2 = ScrewTrajectory(B, B0, Tf, N, method);
traj3 = ScrewTrajectory(B0, B, Tf, N, method);
traj4 = ScrewTrajectory(B, C, Tf, N, method);
traj5 = ScrewTrajectory(C, C0, Tf, N, method);
traj6 = ScrewTrajectory(C0, C, Tf, N, method);
s=1;
j=1;
k=1;
f=1;
m=1;
n=1;



for s=1:N
CSV(s,:)=[traj{1,s}(1,1),traj{1,s}(1,2),traj{1,s}(1,3),traj{1,s}(2,1),traj{1,s}(2,2),traj{1,s}(2,3),traj{1,s}(3,1),traj{1,s}(3,2),traj{1,s}(3,3),traj{1,s}(1,4),traj{1,s}(2,4),traj{1,s}(3,4),0];
s=s+1; 
end
for j=1:N
CSV(N+j,:)=[traj2{1,j}(1,1),traj2{1,j}(1,2),traj2{1,j}(1,3),traj2{1,j}(2,1),traj2{1,j}(2,2),traj2{1,j}(2,3),traj2{1,j}(3,1),traj2{1,j}(3,2),traj2{1,j}(3,3),traj2{1,j}(1,4),traj2{1,j}(2,4),traj2{1,j}(3,4),0];
j=j+1;
end

for k=1:N
CSV(2*N+k,:)=[traj3{1,k}(1,1),traj3{1,k}(1,2),traj3{1,k}(1,3),traj3{1,k}(2,1),traj3{1,k}(2,2),traj3{1,k}(2,3),traj3{1,k}(3,1),traj3{1,k}(3,2),traj3{1,k}(3,3),traj3{1,k}(1,4),traj3{1,k}(2,4),traj3{1,k}(3,4),0];
k=k+1;
end

for f=1:N
CSV(3*N+f,:)=[traj4{1,f}(1,1),traj4{1,f}(1,2),traj4{1,f}(1,3),traj4{1,f}(2,1),traj4{1,f}(2,2),traj4{1,f}(2,3),traj4{1,f}(3,1),traj4{1,f}(3,2),traj4{1,f}(3,3),traj4{1,f}(1,4),traj4{1,f}(2,4),traj4{1,f}(3,4),0];
f=f+1;
end

for m=1:N
CSV(4*N+m,:)=[traj5{1,m}(1,1),traj5{1,m}(1,2),traj5{1,m}(1,3),traj5{1,m}(2,1),traj5{1,m}(2,2),traj5{1,m}(2,3),traj5{1,m}(3,1),traj5{1,m}(3,2),traj5{1,m}(3,3),traj5{1,m}(1,4),traj5{1,m}(2,4),traj5{1,m}(3,4),0];
m=m+1;
end


for n=1:N
CSV(5*N+n,:)=[traj6{1,n}(1,1),traj6{1,n}(1,2),traj6{1,n}(1,3),traj6{1,n}(2,1),traj6{1,n}(2,2),traj6{1,n}(2,3),traj6{1,n}(3,1),traj6{1,n}(3,2),traj6{1,n}(3,3),traj6{1,n}(1,4),traj6{1,n}(2,4),traj6{1,n}(3,4),0];
n=n+1;
end
CSV;
end
 
% Z=[CSV];
% 
% writematrix(Z,'Trajectory.csv','Delimiter',',','QuoteStrings',false);
% type 'Trajectory.csv'
%y=CSV;
%writematrix(y,'filename.csv')
% csvwrite ('endeffector.csv',y);


