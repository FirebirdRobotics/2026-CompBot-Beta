// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FloorRollerConstants;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FloorRollers extends SubsystemBase {
  /** Creates a new FloorRollers. */

  private final TalonFX m_floorroller = new TalonFX(FloorRollerConstants.floorRollerMotorCANID, "CANivore"); // change to different motor, idk what the motor is supposed to be

  public FloorRollers() {

    var rollerMotorConfigs = new TalonFXConfiguration();

    rollerMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  
    rollerMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;
    rollerMotorConfigs.CurrentLimits.StatorCurrentLimit = 50;
  
    rollerMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerMotorConfigs.CurrentLimits.StatorCurrentLimit = 50;
  
    rollerMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;
  
    m_floorroller.getConfigurator().apply(rollerMotorConfigs);
  }

  @Override
  public void periodic() {
    
  }

  public void setRollerMotorPercentOutput(double outputPercent) {
    m_floorroller.setControl(new DutyCycleOut(outputPercent));
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
}
