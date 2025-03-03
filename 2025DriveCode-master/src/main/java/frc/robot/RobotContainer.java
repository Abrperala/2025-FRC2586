// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.util.Units;

import java.lang.management.OperatingSystemMXBean;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.SequenceWriter;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; 
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AdvToShooter;
import frc.robot.commands.HomeElevator;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.PIDElevator;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
//import frc.robot.commands.LiftElevToHallEffect;
import frc.robot.commands.HoldElev;
import frc.robot.commands.ShootCoral;
import frc.robot.commands.StartRampWheel;
import frc.robot.commands.StartShooterWheel;
import frc.robot.commands.StopRampWheel;
import frc.robot.commands.StopShooterWheel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RampSubSystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WristSubSystem;
import frc.robot.subsystems.WristSubSystem.WristPosition;


public class RobotContainer {


    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final PS4Controller driverJoystick = new PS4Controller(0);
    private final PS4Controller operatorJoystick = new PS4Controller(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final WristSubSystem wristSubsystem = new WristSubSystem();
    private final RampSubSystem rampSubsystem = new RampSubSystem();
    private final Shooter shooter = new Shooter();



    public RobotContainer() {
        configureBindings();
    }

    private double squareInput(double input){
        boolean negative = input  < 0;
        if (negative){
            return -Math.pow(input,2);
        }
        else{
            return Math.pow(input,2);
        }
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-.7*squareInput(driverJoystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-.7*squareInput(driverJoystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-.7*squareInput(driverJoystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        /*                        
         * DRIVER CONTROLLER BINDINGS
        */

        // driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverJoystick.b().whileTrue(drivetrain.applyRequest(() ->
        new JoystickButton(driverJoystick, 2).whileTrue(drivetrain.applyRequest(() -> brake));
        new JoystickButton(driverJoystick, 3).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        new JoystickButton(driverJoystick, 9).and( new JoystickButton(driverJoystick, 4)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        new JoystickButton(driverJoystick, 9).and( new JoystickButton(driverJoystick, 1)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        new JoystickButton(driverJoystick, 10).and( new JoystickButton(driverJoystick, 4)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        new JoystickButton(driverJoystick, 10).and( new JoystickButton(driverJoystick, 1)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        new JoystickButton(driverJoystick, 5).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        /*                        
         * OPERATOR CONTROLLER BINDINGS
        */

        //Bind Operator X button to intake Coral directly to shooter
        new POVButton(operatorJoystick, 270).onTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new AdvToShooter(shooter),
                    new StartRampWheel(rampSubsystem)),
                new ParallelCommandGroup(
                    new StopRampWheel(rampSubsystem),
                    new StartShooterWheel(shooter)),
                new WaitCommand(.1),
                new StopShooterWheel(shooter)));

        //Bind Operator circle button to shoot L1
        new JoystickButton(operatorJoystick, 3).onTrue(
                new PIDElevator(ElevatorPosition.L1, elevatorSubsystem));

        //Bind Operator cross button to shoot L2
        new JoystickButton(operatorJoystick, 2).onTrue(
                new PIDElevator(ElevatorPosition.L2, elevatorSubsystem));

        //Bind Operator square button to shoot L3
        new JoystickButton(operatorJoystick, 1).onTrue(
            new PIDElevator(ElevatorPosition.L3, elevatorSubsystem));

        //Bind Operator triange button to shoot L4 *****JUST USE TOP LIMIT SWITCH*******
        //Bind Operator square button to shoot L3
        new JoystickButton(operatorJoystick, 4).onTrue(
            new PIDElevator(ElevatorPosition.L4, elevatorSubsystem));
            
            new JoystickButton(operatorJoystick, 14).onTrue(
                new PIDElevator(ElevatorPosition.Home, elevatorSubsystem));

                new JoystickButton(operatorJoystick, 14).onFalse(
                    new RunCommand(() -> elevatorSubsystem.setMotorSpeed(0)));
    
            new JoystickButton(operatorJoystick, 13).onTrue(
                new ShootCoral(shooter));


        // new POVButton(operatorJoystick, 90).onTrue(
        //     new SequentialCommandGroup(
        //         //new liftElevToHallEffect(elevatorSubsystem, 1),
        //         //new angleWrist(WristSubSystem.WristPosition.ALGAEPICKUP),
        //         new IntakeAlgae(shooter)));
        //         //new angleWrist(WristSubSystem.WristPosition.HOME),
        //         //new HomeElevator(elevatorSubsystem)));

        //Bind triggers for elevator.
        // new JoystickButton(operatorJoystick, 8).whileTrue(new RunCommand(() ->
        // {
        //     elevatorSubsystem.setMotorSpeed(0.5*0.1*(1 + operatorJoystick.getRawAxis(4)));
        //     SmartDashboard.putNumber("Elevator right trigger", operatorJoystick.getRawAxis(4));
        // }))
        // .onFalse(new RunCommand(() -> elevatorSubsystem.setMotorSpeed(0)));
        // new JoystickButton(operatorJoystick, 7).whileTrue(new RunCommand(() -> 
        // {
        //     elevatorSubsystem.setMotorSpeed(0.5*-0.1*(1 + operatorJoystick.getRawAxis(3)));
        //     SmartDashboard.putNumber("Elevator left trigger", operatorJoystick.getRawAxis(3));
        // }))
        // .onFalse(new RunCommand(() -> elevatorSubsystem.setMotorSpeed(0)));
        
        //Bind buttons for the shooter
        new JoystickButton(operatorJoystick, 5).whileTrue(new RunCommand(() -> shooter.setShooterSpeed(.4), shooter))
                                     .whileFalse(new RunCommand(()-> shooter.stopShooter(), shooter));
        new JoystickButton(operatorJoystick, 6).whileTrue(new RunCommand(() -> shooter.setShooterSpeed(-.4), shooter))
                                      .whileFalse(new RunCommand(()-> shooter.stopShooter(), shooter));
        
        //Bind buttons for the ramp motor
        // new POVButton(operatorJoystick, 0).whileTrue(new RunCommand(() -> rampSubsystem.setRampSpeed(0.2)))
        //                     .whileFalse(new RunCommand(() -> rampSubsystem.setRampSpeed(0.0)));
        
        // new POVButton(operatorJoystick, 180).whileTrue(new RunCommand(() -> rampSubsystem.setRampSpeed(-0.2)))
        //                     .whileFalse(new RunCommand(() -> rampSubsystem.setRampSpeed(0.0)));
        
        //Test code for wrist with joystick.
        new JoystickButton(operatorJoystick, 8).whileTrue(new RunCommand(() -> {
            //run much slower than the input with "0.05*"
            wristSubsystem.setSpeed(0.1*operatorJoystick.getRawAxis(1));
            SmartDashboard.putNumber("Wrist Joystick Input", operatorJoystick.getRawAxis(1));
        }));

      

        //
        
        // Bind buttons for the wrist subsystem positions
        //operatorJoystick.x().whileTrue(new RunCommand(() -> wristSubsystem.setWristPosition(WristPosition.LOW)));
        //operatorJoystick.a().whileTrue(new RunCommand(() -> wristSubsystem.setWristPosition(WristPosition.MIDDLE)));
        //operatorJoystick.b().whileTrue(new RunCommand(() -> wristSubsystem.setWristPosition(WristPosition.HIGH)));

        // Bind the Xbox controller triggers to control the elevator
         // I think I want the TriggerAxis value, not the boolean
        /* 
        new Trigger(() -> joystick.getRightTriggerAxis() > 0.1)
            .whileTrue(new RunCommand(() -> elevatorSubsystem.setMotorSpeed(joystick.getRightTriggerAxis()), elevatorSubsystem));

        new Trigger(() -> joystick.getLeftTriggerAxis() > 0.1)
            .whileTrue(new RunCommand(() -> elevatorSubsystem.setMotorSpeed(-joystick.getLeftTriggerAxis()), elevatorSubsystem));
        */
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
