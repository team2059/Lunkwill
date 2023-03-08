// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtendArm extends SubsystemBase {

  public CANSparkMax getExtensionMotor() {
    return extensionMotor;
  }

  public RelativeEncoder getExtensionEncoder() {
    return extensionEncoder;
  }

  public PIDController getExtensionController() {
    return extensionController;
  }

  public double getExtendPosition() {
    return extensionEncoder.getPosition();
  }

  public CANSparkMax extensionMotor;
  public RelativeEncoder extensionEncoder;

  public PIDController extensionController;

  public DigitalInput limitSwitch;
  public boolean isLimitReached;
  public Servo servo;

  public boolean isLimitReached() {
    return isLimitReached;
  }

  public DigitalInput getLimitSwitch() {
    return limitSwitch;
  }

  /** Creates a new ExampleSubsystem. */
  public ExtendArm() {

    extensionMotor = new CANSparkMax(Constants.ArmConstants.extendId, MotorType.kBrushless);

    extensionEncoder = extensionMotor.getEncoder();
    limitSwitch = new DigitalInput(9);
    servo = new Servo(9);

    extensionController = new PIDController(Constants.ArmConstants.extensionkP, 0, 0.001);

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */

    extensionMotor.restoreFactoryDefaults();
    extensionMotor.setIdleMode(IdleMode.kBrake);
    extensionMotor.setInverted(false);
    // extensionEncoder.setPosition(0);

  }

  public void setServoAngle(double angle) {
    servo.setAngle(angle);
  }

  @Override
  public void periodic() {

    isLimitReached = !limitSwitch.get();

    SmartDashboard.putNumber("extension pos", extensionEncoder.getPosition());
    SmartDashboard.putBoolean("limit switch", isLimitReached);
    if (isLimitReached) {
      // System.out.println("isLimitReached = " + isLimitReached);
      extensionEncoder.setPosition(0);
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
