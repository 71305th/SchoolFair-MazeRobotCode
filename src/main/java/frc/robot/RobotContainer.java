// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LimelightAim;
import frc.robot.commands.AbsoluteAim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final LimeLightSubsystem m_limelight = new LimeLightSubsystem();

    // The driver's controller
    Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () -> m_robotDrive.arcadeDrive(
                                -0.4*m_driverController.getRawAxis(1), 0.6*m_driverController.getRawAxis(4)),
                        m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Drive at half speed when the right bumper is held
        // new JoystickButton(m_driverController, 6)
        //         .whenPressed(() -> m_robotDrive.setMaxOutput(0.4))
        //         .whenReleased(() -> m_robotDrive.setMaxOutput(0.8));
        new JoystickButton(m_driverController, 1).toggleWhenPressed(new LimelightAim(m_robotDrive, m_limelight));
        // new JoystickButton(m_driverController, 2).whenPressed(()->CommandScheduler.getInstance().cancelAll());
        // new JoystickButton(m_driverController, 4).whenPressed(()->m_robotDrive.resetOdometry(new Pose2d(0,0,new Rotation2d(0))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        //         new SimpleMotorFeedforward(
        //                 DriveConstants.ksVolts,
        //                 DriveConstants.kvVoltSecondsPerMeter,
        //                 DriveConstants.kaVoltSecondsSquaredPerMeter),
        //         DriveConstants.kDriveKinematics,
        //         10);

        // // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        //         AutoConstants.kMaxSpeedMetersPerSecond,
        //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                 // Add kinematics to ensure max speed is actually obeyed
        //                 .setKinematics(DriveConstants.kDriveKinematics)
        //                 // Apply the voltage constraint
        //                 .addConstraint(autoVoltageConstraint);

        // // An example trajectory to follow. All units in meters.
        // Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         // Pass through these two interior waypoints, making an 's' curve path
        //         List.of(
        //                 new Translation2d(1, 0),
        //                 new Translation2d(2, 0),
        //                 new Translation2d(3, 0),
        //                 new Translation2d(3.1,0.1),
        //                 new Translation2d(3.2,0.2),
        //                 new Translation2d(3.3,0.3),
        //                 new Translation2d(3.4,0.4),
        //                 new Translation2d(3.5,0.5)
        //                 // new Translation2d(0.5,0.5),
        //                 // new Translation2d(1,1),
        //                 // new Translation2d(1,2)
        //         ),
        //         new Pose2d(3.5, 1, new Rotation2d(0)),
        //         config);
        // String trajectoryJSON = "paths/testPath.wpilib.json";
        // try{
        // Path jsonPath =
        // Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        // Trajectory testTrajectory = TrajectoryUtil.fromPathweaverJson(jsonPath);
        // Command ramseteCommand = m_robotDrive.createCommandForTrajectory(testTrajectory);

        // // Reset odometry to the starting paose of the trajectory.
        // m_robotDrive.resetOdometry(testTrajectory.getInitialPose());

        // // Run path following command, then stop at the end.
        // return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));
        // } catch (IOException e){
        // DriverStation.reportError("can't open json:"+e.getMessage(),
        // e.getStackTrace());
        // }
        return new RunCommand(()->CommandScheduler.getInstance().schedule(new LimelightAim(m_robotDrive, m_limelight)),m_robotDrive,m_limelight);
    }

    public void testDrive(){
        m_robotDrive.testMotor();
    }
    public void stopTest(){
        m_robotDrive.stopMotor();
    }
}
