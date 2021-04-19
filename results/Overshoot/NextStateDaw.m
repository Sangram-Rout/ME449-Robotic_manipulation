function [Vec_robconfig_dt] = NextStateDaw(CurConfig,Speed,dt,MaxAS)
Chassis_config = CurConfig(1:3);
Arm_Config = CurConfig(4:8);
Wheel_Config = CurConfig(9:12);
w = 0.15;
l = 0.235;
r = 0.0475;
H = (r/4)*[[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)];
           [1,1,1,1];
           [-1,1,-1,1]];

Arm_Speed = Speed(1:5);
Wheel_Speed = Speed(6:9);

 for i = 1 : 5
     if Arm_Speed(i)>MaxAS||Arm_Speed(i)<(-1)*MaxAS
         Arm_Speed(i) = MaxAS;
     end
 end
 for n = 1 : 4
     if Wheel_Speed(n)>MaxAS||Wheel_Speed(n)<(-1)*MaxAS
         Wheel_Speed(n) = MaxAS;
     end
 end

[New_ArmJA, New_ArmJS] = EulerStep(Arm_Config', Arm_Speed',0, dt);

[New_WheelA, New_WheelS] = EulerStep(Wheel_Config', Wheel_Speed',0, dt);



Vb = (H*Wheel_Speed');

Odo = Chassis_config';
[New_Odo, New_OdoS] = EulerStep(Odo, Vb, 0, dt);

Vec_robconfig_dt = [New_Odo',New_ArmJA',New_WheelA'];
end




