// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PowerDistributionPanel extends SubsystemBase {
  PowerDistribution revPDP;

  /** Creates a new PowerDistribution. */
  public PowerDistributionPanel() {
    revPDP = new PowerDistribution(Constants.PowerDistribution.pdpID, ModuleType.kRev);

  }

  @Override
  public void periodic() {
    boolean hasStickyFaults = false;

    // 0-4
    // 15,19,13,20-23

    if (revPDP.getStickyFaults().Channel0BreakerFault || revPDP.getStickyFaults().Channel1BreakerFault
        || revPDP.getStickyFaults().Channel2BreakerFault || revPDP.getStickyFaults().Channel3BreakerFault
        || revPDP.getStickyFaults().Channel4BreakerFault
        || revPDP.getStickyFaults().Channel13BreakerFault || revPDP.getStickyFaults().Channel15BreakerFault
        || revPDP.getStickyFaults().Channel16BreakerFault || revPDP.getStickyFaults().Channel17BreakerFault
        || revPDP.getStickyFaults().Channel18BreakerFault || revPDP.getStickyFaults().Channel19BreakerFault
        || revPDP.getStickyFaults().Channel20BreakerFault || revPDP.getStickyFaults().Channel21BreakerFault
        || revPDP.getStickyFaults().Channel22BreakerFault
        || revPDP.getStickyFaults().Channel23BreakerFault) {
      hasStickyFaults = true;
    } else {
      hasStickyFaults = false;
    }

    SmartDashboard.putBoolean("hasStickyFaults", hasStickyFaults);
    SmartDashboard.putData("reset sticky faults", new InstantCommand(() -> revPDP.clearStickyFaults()));

    // This method will be called once per scheduler run
  }
}
