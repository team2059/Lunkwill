// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {

  public CANSparkMax getTiltMotor() {
    return tiltMotor;
  }

  public CANSparkMax getExtensionMotor() {
    return extensionMotor;
  }

  public RelativeEncoder getExtensionEncoder() {
    return extensionEncoder;
  }

  public PIDController getExtensionController() {
    return extensionController;
  }

  public DutyCycleEncoder getThruBoreEncoder() {
    return thruBoreEncoder;
  }

  public double getThruBorePosition() {
    return thruBoreEncoder.getAbsolutePosition();
  }

  public double getExtendPosition() {
    return extensionEncoder.getPosition();
  }

  public PIDController getTiltController() {
    return tiltController;
  }

  public CANSparkMax tiltMotor, extensionMotor;
  public RelativeEncoder extensionEncoder;
  public DutyCycleEncoder thruBoreEncoder;
  public double thruBorePosition;
  public PIDController tiltController, extensionController;

  /** Creates a new ExampleSubsystem. */
  public Arm() {

    tiltMotor = new CANSparkMax(Constants.ArmConstants.tiltId, MotorType.kBrushless);
    extensionMotor = new CANSparkMax(Constants.ArmConstants.extendId, MotorType.kBrushless);
    thruBoreEncoder = new DutyCycleEncoder(Constants.ArmConstants.thruBoreDIO);
    extensionEncoder = extensionMotor.getEncoder();

    tiltController = new PIDController(Constants.ArmConstants.tiltkP, 0.00, Constants.ArmConstants.tiltkD);
    extensionController = new PIDController(Constants.ArmConstants.extensionkP, 0, 0);
    /**
     * The restoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    tiltMotor.restoreFactoryDefaults();
    tiltMotor.setIdleMode(IdleMode.kBrake);
    tiltMotor.setInverted(true);

    extensionMotor.restoreFactoryDefaults();
    extensionMotor.setIdleMode(IdleMode.kBrake);
    extensionMotor.setInverted(true);
    // extensionEncoder.setPosition(0);

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("thru bore pos", thruBoreEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("extension pos", extensionEncoder.getPosition());
    double tiltOutput = RobotContainer.driver.getRawAxis(5) * 0.5;
    double extendOutput = RobotContainer.driver.getRawAxis(1) * 0.5;
    if (Math.abs(tiltOutput) < 0.05) {
      tiltOutput = 0;
    }
    if (Math.abs(extendOutput) < 0.05) {
      extendOutput = 0;
    }
    // System.out.println(output);
    // extensionMotor.set(extendOutput);
    // tiltMotor.set(tiltOutput);

  }

  // public void manualTiltGoToPos() {
  // if (Math.abs(RobotContainer.driver.getRawAxis(1)) < 0.1) {
  // tiltMotor.set(0);
  // return;
  // }
  // tiltMotor.set(0.66 * RobotContainer.driver.getRawAxis(1));
  // tiltMotor.setIdleMode(IdleMode.kBrake);
  // return;
  // }

  // public void PIDGoToPos(double setpoint) {
  // double output =
  // tiltController.calculate(thruBoreEncoder.getAbsolutePosition(), setpoint);
  // SmartDashboard.putNumber("output", output);
  // System.out.println("OUT PUT" + output);

  // while (RobotContainer.driver.getRawButton(0)) {
  // // figure out why output needs to be negated
  // tiltMotor.set(-output);
  // return;
  // }
  // return;
  // }

  // public void manualArmExtension() {
  // if (Math.abs(RobotContainer.driver.getRawAxis(5)) < 0.1) {
  // extensionMotor.set(0);
  // return;
  // }
  // extensionMotor.set(0.66 * RobotContainer.driver.getRawAxis(5));
  // extensionMotor.setIdleMode(IdleMode.kBrake);
  // return;
  // }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
