// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Shooter extends SubsystemBase {
  // Need to configure CAN ID's
  private final TalonFX m_leader = new TalonFX(ShooterConstants.shooterLeaderMotorID, "CANivore");
  private final TalonFX m_follower = new TalonFX(ShooterConstants.shooterFollowerMotorID, "CANivore");

  


  /** Creates a new Shooter. */
  public Shooter() {
    // Set follower motor to follow leader
    m_follower.setControl(new Follower(m_leader.getDeviceID(), MotorAlignmentValue.Opposed));



    var shooterMotorConfigs = new TalonFXConfiguration();

    shooterMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    shooterMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    shooterMotorConfigs.Feedback.SensorToMechanismRatio = ((1));

    shooterMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterMotorConfigs.CurrentLimits.StatorCurrentLimit = 50;

    shooterMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;



    // set slot 0 gains
    var slot0Configs = shooterMotorConfigs.Slot0;
    slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative    


    

    
    // set Motion Magic settings
    var motionMagicConfigs = shooterMotorConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 30; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 0.0; // Target jerk of 1600 rps/s/s (0.1 seconds)

    m_leader.getConfigurator().apply(shooterMotorConfigs);

  }

  // spins the shooter to RPM
  public void goToRPM(double RPM){

    // create a velocity closed-loop request, voltage output, slot 0 configs
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    // set velocity to 8 rps, add 0.5 V to overcome gravity
    m_leader.setControl(m_request.withVelocity(8).withFeedForward(0.5));

  }
  
  // percent should be a value between -1 and 1, representing the percentage of max voltage to apply to the motor
  public void setPercentOutput(double percent) {
    m_leader.setControl(new DutyCycleOut(percent));
  }

  public Command setVelocityCommand(double RPM) {

    return runOnce(() -> goToRPM(RPM));


  }

  
  public Command setPercentOutputCommand(double percent) {
    return runOnce(() -> setPercentOutput(percent));
  
  }

  public Command goTo400RPM() {

    return runOnce(() -> goToRPM(400));

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // DogLog.log("Elevator position", m_leader.getPosition().getValueAsDouble());

  }


@Override
public void simulationPeriodic() {

}

}