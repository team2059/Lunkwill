// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final VictorSPX indexerMotor;
  private final CANSparkFlex topDriveMotor, bottomDriveMotor;


  /** Creates a new Shooter. */
  public Shooter(int indexerMotorId, int topDriveMotorId, int bottomDriveMotorId) {
    indexerMotor = new VictorSPX(indexerMotorId);

    topDriveMotor = new CANSparkFlex(topDriveMotorId, MotorType.kBrushless);
    bottomDriveMotor = new CANSparkFlex(bottomDriveMotorId, MotorType.kBrushless);

    topDriveMotor.follow(bottomDriveMotor);
  }

  public void setDriveMotorsSpeed(double speed) {
    bottomDriveMotor.set(speed);
  }

  public void setIndexerMotorSpeed(double speed) {
    indexerMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public void stopAllMotors() {
    bottomDriveMotor.set(0);
    indexerMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void setDriveMotorsMode(boolean brake) {
    if (brake) {
      bottomDriveMotor.setIdleMode(IdleMode.kBrake);
      topDriveMotor.setIdleMode(IdleMode.kBrake);
    } else {
      bottomDriveMotor.setIdleMode(IdleMode.kCoast);
      topDriveMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  @Override
  public void periodic() {
  }
}
