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
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final TalonFX m_pivotMotor = new TalonFX(IntakeConstants.pivotMotorCANID, "CANivore");
  private final TalonFX m_rollerMotor = new TalonFX(IntakeConstants.rollerMotorCANID, "CANivore");

  /** Creates a new Intake. 
 * @param m_Leds */
  public Intake(LEDs m_Leds) {
    var pivotMotorConfigs = new TalonFXConfiguration();
    
    pivotMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    pivotMotorConfigs.Feedback.SensorToMechanismRatio = 6.333;

    pivotMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;
    pivotMotorConfigs.CurrentLimits.StatorCurrentLimit = 50;


    pivotMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotMotorConfigs.CurrentLimits.StatorCurrentLimit = 50;

    pivotMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;

    // set slot 0 gains
    var slot0Configs = pivotMotorConfigs.Slot0;
    slot0Configs.kG = 0.0; // Probably don't need kg
    slot0Configs.kS = 0.18; // Add 0.25 V output to overcome static friction /*VALUE HAS BEEN TUNED  */
    slot0Configs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.0; // A positio n error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static; //FRC discord smart people say to use this

    var motionMagicConfigs = pivotMotorConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 30.0; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 30.0; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 0.0; // Target jerk of 1600 rps/s/s (0.1 seconds)

    m_pivotMotor.getConfigurator().apply(pivotMotorConfigs);

    var intakeRollerConfigs = new TalonFXConfiguration();
    intakeRollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeRollerConfigs.CurrentLimits.StatorCurrentLimit = 50;

    intakeRollerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeRollerConfigs.CurrentLimits.SupplyCurrentLimit = 50;

    m_rollerMotor.getConfigurator().apply(intakeRollerConfigs);

  }

  public void extendToDistance(double inches) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(inches);

    m_pivotMotor.setControl(m_request.withPosition(inches));
  }

    public Command goToDeployAndThenToUndeployCommand() {
    return runEnd(
      () -> goToDeployedPosition(),
      () -> goToFramePerimeterPosition()
      
      ); 
  }

  public void goToDeployedPosition() {

    final MotionMagicVoltage m_request = new MotionMagicVoltage(IntakeConstants.deployDistance);

    m_pivotMotor.setControl(m_request.withPosition(IntakeConstants.deployDistance));

    setRollerMotorPercentOutput(0.8);

  }

  public void goToFramePerimeterPosition() {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(IntakeConstants.framePerimeterDistance);

    m_pivotMotor.setControl(m_request.withPosition(IntakeConstants.framePerimeterDistance));

    setRollerMotorPercentOutput(0);
  }

  public Command CommandGoToDistance(double inches) {
    return new InstantCommand(()-> extendToDistance(inches), this);
  }

  public void setRollerMotorPercentOutput(double outputPercent) {
    m_rollerMotor.setControl(new DutyCycleOut(outputPercent));
  }

  public Command setRollerMotorPercentOutputAndThenTo0Command(double power) {
    
    return runEnd(
      () -> setRollerMotorPercentOutput(power),
      () -> setRollerMotorPercentOutput(0)
    ); 

  }

  public Command setRollerMotorPercentOutputCommand(double power) {
    return runOnce(() -> setRollerMotorPercentOutput(power));
    

  }

  public Command goToDeployedPositionCommand() {
    return runOnce(() -> goToDeployedPosition());
    
  }

  public Command goToFramePerimeterPositionCommand() {
    return runOnce(() -> goToFramePerimeterPosition());
    
  }

  public Command goToMidPointPositionCommand() {
    return runOnce(() -> extendToDistance(IntakeConstants.midPoint));
    
  }

  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
