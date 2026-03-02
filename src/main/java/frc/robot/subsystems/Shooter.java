// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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
  private final TalonFX m_leader = new TalonFX(ShooterConstants.shooterLeaderMotorID, "rio");
  private final TalonFX m_follower = new TalonFX(ShooterConstants.shooterFollowerMotorID, "rio");

  


  /** Creates a new Shooter. */
  public Shooter() {
    // Set follower motor to follow leader
    m_follower.setControl(new Follower(m_leader.getDeviceID(), MotorAlignmentValue.Opposed));

     /* The follower motor should be following the leader motors current requests 
     at any given time so there is no need to configure a follower motor beyond the above line*/


    var shooterLeaderMotorConfigs = new TalonFXConfiguration();

    shooterLeaderMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    shooterLeaderMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    shooterLeaderMotorConfigs.Feedback.SensorToMechanismRatio = ((1)); //Gear ratio is 1 to 1

    shooterLeaderMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterLeaderMotorConfigs.CurrentLimits.StatorCurrentLimit = 50;

    shooterLeaderMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterLeaderMotorConfigs.CurrentLimits.SupplyCurrentLimit = 50;


    // set slot 0 gains
    var slot0Configs = shooterLeaderMotorConfigs.Slot0;
    slot0Configs.kS = 0.0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.0; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative
    

    

    
    // set Motion Magic settings
    var motionMagicConfigs = shooterLeaderMotorConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    // motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)
    m_leader.getConfigurator().apply(shooterLeaderMotorConfigs);


  }

  // spins the shooter to RPS
  public void goToRPS(double RPS){
    
    // create a Motion Magic Velocity request, voltage output
    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(RPS);

    m_leader.setControl(m_request.withVelocity(RPS));

  }
  
  // percent should be a value between -1 and 1, representing the percentage of max voltage to apply to the motor
  public void setPercentOutput(double percent) {
    m_leader.setControl(new DutyCycleOut(percent));
  }

  public Command setVelocityCommand(double RPS) {

    return runOnce(() -> goToRPS(RPS));


  }

  
  public Command setPercentOutputCommand(double percent) {
    return runOnce(() -> setPercentOutput(percent));
  
  }

  public Command goTo400RPS() {

    return runOnce(() -> goToRPS(400));
  
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