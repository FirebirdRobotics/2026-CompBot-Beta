// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HoodConstants;

public class Hood extends SubsystemBase {
    private final TalonFX m_pivotMotor = new TalonFX(HoodConstants.hoodPivotMotorID, "rio");

    
  /** Creates a new Hood. */
  public Hood() {
    var pivotMotorConfigs = new TalonFXConfiguration();
    
    pivotMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // This is correct
    pivotMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // Confirm with omri that sensor to mech is correct
    pivotMotorConfigs.Feedback.SensorToMechanismRatio = 4.0;

    pivotMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;
    pivotMotorConfigs.CurrentLimits.StatorCurrentLimit = 50;


    pivotMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotMotorConfigs.CurrentLimits.StatorCurrentLimit = 50;

    pivotMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;

    pivotMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    // set slot 0 gains
    var slot0Configs = pivotMotorConfigs.Slot0;
    slot0Configs.kG = 0.25; // 
    slot0Configs.kS = 0.0; // Add 0.25 V output to overcome static friction /*VALUE HAS BEEN TUNED  */
    slot0Configs.kV = 0.85; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.1; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 45; // A positio n error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine; 
        

    var motionMagicConfigs = pivotMotorConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 10.0; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 10.0; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 0.0; // Target jerk of 1600 rps/s/s (0.1 seconds)

    m_pivotMotor.getConfigurator().apply(pivotMotorConfigs);

  }

  public void setDutyCycleOutput(double dutyCycle) {
    m_pivotMotor.setControl(new DutyCycleOut(dutyCycle));
  }

  public Command CommandSetDutyCycleOutput(double dutyCycle) {
    return new InstantCommand(()-> setDutyCycleOutput(dutyCycle), this);
  }

  // Need to get proper angle values
  public void goToAngle(double angle) {
    // final PositionVoltage m_request = new PositionVoltage(angle).withSlot(0);
    final MotionMagicVoltage m_request = new MotionMagicVoltage(angle);

    m_pivotMotor.setControl(m_request.withPosition(angle));
  }

  public Command CommandGoToAngle(double angle) {
    return new InstantCommand(()-> goToAngle(angle), this);
  }

// Double check this later
  public Command CommandGoToLowestAngle() {
    return new InstantCommand(()-> goToAngle(-0.11865), this);
  }


  // Base position (lowest angle possible) -0.11865


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
