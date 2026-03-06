// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.AlignToReef;
import frc.robot.Commands.AlignToReef.ReefSide;
import frc.robot.constants.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DiagonAlley;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.FloorRollers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
// import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TransferRollers;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    CommandGenericHID  buttonBoardRight = new CommandGenericHID(2);
    CommandGenericHID  buttonBoardLeft = new CommandGenericHID(1);


    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    LEDs m_Leds = new LEDs();

    FloorRollers m_FloorRollers = new FloorRollers();

    DiagonAlley m_DiagonAlley = new DiagonAlley();



    Shooter m_Shooter = new Shooter(); 

    Elevator m_Elevator = new Elevator();

    Intake intake = new Intake(m_Leds);

    TransferRollers m_transferRollers = new TransferRollers();


    EndEffector m_EndEffector = new EndEffector(m_Leds);


    // Vision vision;


    
    

    

    public RobotContainer() {

        NamedCommands.registerCommand("L1 Elevator and EE to Proper Positions", Commands.parallel(m_Elevator.goToL1(),m_EndEffector.goToL1()));
        NamedCommands.registerCommand("L3 Elevator and EE to Proper Positions", Commands.parallel(m_Elevator.goToL3(),m_EndEffector.goToL3()));

        NamedCommands.registerCommand("L4 Elevator and EE to Proper Positions", Commands.parallel(m_Elevator.goToL4(),m_EndEffector.goToL4()));

        NamedCommands.registerCommand("Outake Coral", m_EndEffector.setRollerMotorPercentOutputAndThenTo0Command( 0.35).withTimeout(3));

        FollowPathCommand.warmupCommand().schedule();

        

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // vision = new Vision(drivetrain);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );



        joystick.a().whileTrue(m_Shooter.testShooter());


        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // drivetrain.registerTelemetry(logger::telemeterize);

        // joystick.x().onTrue(Commands.parallel(m_transferRollers.manualRollBackward(0.1), m_DiagonAlley.rollOutwards(0.1), m_FloorRollers.rollInwardsCommand(0.1)));
        // joystick.x().onFalse(Commands.parallel(m_transferRollers.manualRollForwards(0), m_DiagonAlley.rollInwardsCommand(0), m_FloorRollers.rollInwardsCommand(0)));
        // ^^^^^UNCOMENT ABOVE WHEN DIAGON ALLEY ROLLERS ARE  REATTATCHED, TEMPORARILY COMMENTING OUT BECAUSE SCREWs DETATCHED^^^^^

        joystick.x().onTrue(Commands.parallel(m_transferRollers.manualRollBackward(0.1), m_FloorRollers.rollInwardsCommand(0.1)));
        joystick.x().onFalse(Commands.parallel(m_transferRollers.manualRollForwards(0), m_FloorRollers.rollInwardsCommand(0)));

        joystick.y().onTrue(m_transferRollers.manualRollBackward(0.5));
        joystick.y().onFalse(m_transferRollers.manualRollBackward(0));






        buttonBoardRight.button(7).whileTrue(Commands.sequence(intake.setRollerMotorPercentOutputAndThenTo0Command(-0.25), intake.goToFramePerimeterPositionCommand()));

        
    }




    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
        // return Commands.sequence(intake.goToFramePerimeterPositionCommand(),drivetrain.driveOutSimpleCommand());
    }
}
