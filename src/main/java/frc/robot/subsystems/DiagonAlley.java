// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DiagonAlleyConstants;

public class DiagonAlley extends SubsystemBase {
  /** Creates a new DiagonAlley. */

    private final TalonFX m_leader = new TalonFX(DiagonAlleyConstants.rightRollerMotorCANID, "rio"); 
    private final TalonFX m_follower = new TalonFX(DiagonAlleyConstants.leftRollerMotorCANID, "rio");

  
  public DiagonAlley() {
    m_follower.setControl(new Follower(m_leader.getDeviceID(), MotorAlignmentValue.Opposed));

    var rollerMotorConfigs = new TalonFXConfiguration();

    rollerMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  
    rollerMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;
    rollerMotorConfigs.CurrentLimits.StatorCurrentLimit = 50;
  
    rollerMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerMotorConfigs.CurrentLimits.StatorCurrentLimit = 50;
  
    rollerMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;
  
    m_leader.getConfigurator().apply(rollerMotorConfigs);

  }

  public void setRollerMotorPercentOutput(double outputPercent) {
    m_leader.setControl(new DutyCycleOut(outputPercent));
  }
  
  public Command StartTurn(double power) {
    return new InstantCommand(
      () -> setRollerMotorPercentOutput(power)
    );
  }

  public Command rollOutwards(double power) {
    return new InstantCommand(
      () -> setRollerMotorPercentOutput(power * -1)
    );
  }

  public Command rollInwardsCommand(double power) {
    return new InstantCommand(
      () -> setRollerMotorPercentOutput(power)
    );
  }
  
  public Command Break(double power) {
    return new InstantCommand(
      () -> setRollerMotorPercentOutput(0)
      );
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
