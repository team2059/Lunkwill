// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

//import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterTest extends SubsystemBase {

  // public CANSparkMax getTiltMotor() {
  // return tiltMotor;
  // }

  // public DutyCycleEncoder getThruBoreEncoder() {
  // return thruBoreEncoder;
  // }

  // public double getThruBorePosition() {
  // return thruBoreEncoder.getAbsolutePosition();
  // }

  // public PIDController getTiltController() {
  // return tiltController;
  // }

  public CANSparkMax shooter9;
  public CANSparkMax shooter10;

  // public DutyCycleEncoder thruBoreEncoder;
  // public double thruBorePosition;
  // public PIDController tiltController;

  /** Creates a new ExampleSubsystem. */
  public ShooterTest() {

    shooter9 = new CANSparkMax(9, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    shooter10 = new CANSparkMax(10, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    // thruBoreEncoder = new DutyCycleEncoder(Constants.ArmConstants.thruBoreDIO);

    // tiltController = new PIDController(Constants.ArmConstants.tiltkP, 0.00,
    // Constants.ArmConstants.tiltkD);
    // tiltController.enableContinuousInput(0, 1);

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    shooter9.restoreFactoryDefaults();
    // tiltMotor.setIdleMode(IdleMode.kBrake);
    shooter9.setInverted(false);

    shooter10.restoreFactoryDefaults();
    shooter9.setInverted(false);

    // extensionEncoder.setPosition(0);

  }

  @Override
  public void periodic() {
    double value = RobotContainer.logitech.getRawAxis(3);
    value = 0 + ((Math.abs(value - 1)) / 2.0);
    SmartDashboard.putNumber("value", value);
    shooter9.set(value);
    shooter10.set(value);
    // SmartDashboard.putNumber("relative tilt pos",
    // tiltMotor.getEncoder().getPosition());

    // SmartDashboard.putNumber("TILTPERCENT", tiltMotor.getAppliedOutput());

    // SmartDashboard.putNumber("TILTVOLTAGE", tiltMotor.getBusVoltage());
    // SmartDashboard.putNumber("thru bore pos",
    // thruBoreEncoder.getAbsolutePosition());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
