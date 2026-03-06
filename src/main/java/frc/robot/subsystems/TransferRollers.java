// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class TransferRollers extends SubsystemBase {
  /** Creates a new TransferRollers. */
  private final TalonFX m_transferRollerMotor = new TalonFX(48, "CANivore"); //Correct
  private final CANrange transferCANrange = new CANrange(100, "CANivore"); //change device ID
  
  public TransferRollers() {
    var transferRollerConfigs = new TalonFXConfiguration();
    transferRollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    transferRollerConfigs.CurrentLimits.StatorCurrentLimit = 50;
    transferRollerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    transferRollerConfigs.CurrentLimits.SupplyCurrentLimit = 50;

    m_transferRollerMotor.getConfigurator().apply(transferRollerConfigs);
  }

  public void setRollerMotorPercentOutput(double outputPercent) {
    m_transferRollerMotor.setControl(new DutyCycleOut(outputPercent));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command manualRollForwards(double power) {
    return new InstantCommand(
      () -> setRollerMotorPercentOutput(power)
    );
  }

  public Command manualRollBackward(double power) {
    return new InstantCommand(
      () -> setRollerMotorPercentOutput(power * -1)
    );
  }

  public boolean checkIfLaser() {
    return transferCANrange.getIsDetected().getValue();
  }

  
  public Command rollUntilLaser(double power) {
    return Commands.run(() -> manualRollForwards(power), this)
        .until(() -> checkIfLaser())
        .finallyDo(interrupted -> manualRollForwards(0));
  }
}
