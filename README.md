# BaseNeoSwerve

FRC Team 2059's Swerve Drive code for the 2022-2023 season.

MK4i Swerve Module with NEO Pinions / L2 6.75:1

Using navX 1 for gyroscope. Might upgrade to navX 2 during build season.
Using CANCoders as well.

TO CHANGE:
Positions in the kinematics are configured differently as we flipped our front two modules by 90 physically. Make changes here if you have them configured in the normal "setup"

Speed is 50% in the TeleopSwerve command. Will definitely change/remove this accordingly, since we have L2 drive gear ratio.

TeleOp 3D april tag PhotonVision settings:
Resolution: 320x240 @ 10 FPS
Stream Resolution: 320x240
target family: tag16h5
target model: 6in (16h5) AprilTag
Decimate: 1
Blur: 0
Therads : 2
Refine Edges = true
max error bits = 0
decision margin cutoff = 30
pose estimation iterations = 100
