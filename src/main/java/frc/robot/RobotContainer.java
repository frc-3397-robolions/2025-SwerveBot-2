package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.States.PositionState;
import frc.robot.Constants.ButtonMap;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    //private final Joystick driver = new Joystick(0);
    // private final CommandXboxController m_driverController = new CommandXboxController(
    //   OperatorConstants.kDriverControllerPort);

    private final CommandJoystick m_operatorController = new CommandJoystick(OperatorConstants.kOperatorControllerPort);

    //private final SendableChooser<Command> autoChooser;

    /* Drive Controls */
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    // private final int strafeAxis = XboxController.Axis.kLeftX.value;
    // private final int rotationXAxis = XboxController.Axis.kRightX.value;
    // private final int rotationYAxis = XboxController.Axis.kRightY.value;

    /* Driver Buttons */
    // private final JoystickButton robotCentric = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    // private final Swerve s_Swerve = new Swerve();
    public final Elevator elevatorMotor1 = new Elevator(ElevatorConstants.CANIDMotor1);
    public final Elevator elevatorMotor2 = new Elevator((ElevatorConstants.CANIDMotor2));
    private final Claw claw = new Claw(elevatorMotor1);
    private PositionState currentState = PositionState.Home;
    private final Intake intake = new Intake();
    // private final UsbCamera frontCamera;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        //autoChooser = AutoBuilder.buildAutoChooser("Tests");
        //SmartDashboard.putData("Auto Mode", autoChooser);

        /*s_Swerve.setDefaultCommand(
            new DriveRobot(
                s_Swerve, 
                () -> -m_driverController.getRawAxis(translationAxis), 
                () -> -m_driverController.getRawAxis(strafeAxis), 
                () -> -m_driverController.getRawAxis(rotationXAxis),
                () -> -m_driverController.getRawAxis(rotationYAxis)
                // () -> robotCentric.getAsBoolean()
            )
        );
        */

        // Configure the button bindings
        configureButtonBindings();

        //Trigger coralDetectorTrigger = new Trigger(() -> intake.getPhotoSensor());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        //m_driverController.button(ButtonMap.Callibration).onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        setButtonAction(ButtonMap.L1, PositionState.Home);
        setButtonAction(ButtonMap.L2, PositionState.L2);
        setButtonAction(ButtonMap.L23, PositionState.L23);
        setButtonAction(ButtonMap.L3, PositionState.L3);
        setButtonAction(ButtonMap.L34, PositionState.L34);
        setButtonAction(ButtonMap.L4, PositionState.L4);
        //setButtonAction(ButtonMap.Barge, PositionState.Barge);
        //setButtonAction(ButtonMap.Floor, PositionState.Floor);
        //setButtonAction(ButtonMap.Processor, PositionState.Processor);

        m_operatorController.button(ButtonMap.cIn).onTrue(intake.Intake_coral());
        m_operatorController.button(ButtonMap.cOut).whileTrue(intake.Outtake());

        //m_operatorController.button(ButtonMap.aIn).whileTrue(intake.Intake_coral());
        //m_operatorController.button(ButtonMap.aOut).whileTrue(intake.Outtake());
    }

    private void setButtonAction(int button, PositionState state)
    {
        m_operatorController.button(button).onTrue(claw.rotateWrist(state));
        m_operatorController.button(button).onTrue(elevatorMotor1.setDesiredHeight(state));
        m_operatorController.button(button).onTrue(elevatorMotor2.setDesiredHeight(state));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return null;//autoChooser.getSelected();
    }
}