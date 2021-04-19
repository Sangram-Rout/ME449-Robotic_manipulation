function [V,Adxixd_Vd,Vd,Xerr,Incre_err] = FeedbackControlDaw(X,Xd,Xd_next,Kp,Ki,dt)
%Milestone 3
%twist
Vd = se3ToVec(MatrixLog6(inv(Xd)*Xd_next)/dt);
Adxixd_Vd = Adjoint(inv(X)*Xd)*Vd;
%Erroe
Xerr = se3ToVec(MatrixLog6(inv(X)*Xd));
%Incrementerroe
Incre_err = Xerr*dt;
V = Adxixd_Vd + Kp*Xerr + Ki*Incre_err;
end

