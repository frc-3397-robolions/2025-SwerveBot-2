package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.States.PositionState;
import frc.robot.Constants.ButtonMap;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    //private final Joystick driver = new Joystick(0);
    private final CommandXboxController m_driverController = new CommandXboxController(
       OperatorConstants.kDriverControllerPort);

    private final CommandJoystick m_operatorController = new CommandJoystick(OperatorConstants.kOperatorControllerPort);
    //private final JoystickButton robotCentric = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    private final SendableChooser<Command> autoChooser;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    public final Elevator elevatorMotor1 = new Elevator(ElevatorConstants.CANIDMotor1);
    public final Elevator elevatorMotor2 = new Elevator((ElevatorConstants.CANIDMotor2));
    private final Claw claw = new Claw(elevatorMotor1);
    private PositionState currentState = PositionState.Home;
    private final Intake intake = new Intake();
    //private final UsbCamera frontCamera;

    public final LEDSubsystem led = new LEDSubsystem();
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //intaking and outtaking coral and algae in auto
        NamedCommands.registerCommand("intakeCoralNamed", intake.Intake_coral().asProxy());
        
        new EventTrigger("outtakeAlgae").whileTrue(intake.Outtake_Algea());
        new EventTrigger("intakeCoral").onTrue(intake.Intake_coral());
        new EventTrigger("intakeAlgae").whileTrue(intake.Intake_Algea());
        new EventTrigger("outtakeCoral").onTrue(intake.Outtake());
        new EventTrigger("outtakeCoralStop").onTrue(intake.OuttakeStop());
        
        //elevator in auto
        new EventTrigger("elevatorHome").onTrue(claw.rotateWrist(PositionState.Home))
                                                                .onTrue(elevatorMotor1.setDesiredHeight(PositionState.Home))
                                                                .onTrue(elevatorMotor2.setDesiredHeight(PositionState.Home));
        new EventTrigger("elevatorL2").onTrue(claw.rotateWrist(PositionState.L2))
                                                                .onTrue(elevatorMotor1.setDesiredHeight(PositionState.L2))
                                                                .onTrue(elevatorMotor2.setDesiredHeight(PositionState.L2));
        new EventTrigger("elevatorL3").onTrue(claw.rotateWrist(PositionState.L3))
                                                                .onTrue(elevatorMotor1.setDesiredHeight(PositionState.L3))
                                                                .onTrue(elevatorMotor2.setDesiredHeight(PositionState.L3));
        new EventTrigger("elevatorL4").onTrue(claw.rotateWrist(PositionState.L4))
                                                                .onTrue(elevatorMotor1.setDesiredHeight(PositionState.L4))
                                                                .onTrue(elevatorMotor2.setDesiredHeight(PositionState.L4));
        new EventTrigger("elevatorL23").onTrue(claw.rotateWrist(PositionState.L23))
                                                                .onTrue(elevatorMotor1.setDesiredHeight(PositionState.L23))
                                                                .onTrue(elevatorMotor2.setDesiredHeight(PositionState.L23));
        new EventTrigger("elevatorL34").onTrue(claw.rotateWrist(PositionState.L34))
                                                                .onTrue(elevatorMotor1.setDesiredHeight(PositionState.L34))
                                                                .onTrue(elevatorMotor2.setDesiredHeight(PositionState.L34));
        new EventTrigger("elevatorProcessor").onTrue(claw.rotateWrist(PositionState.Processor))
                                                                .onTrue(elevatorMotor1.setDesiredHeight(PositionState.Processor))
                                                                .onTrue(elevatorMotor2.setDesiredHeight(PositionState.Processor));
        new EventTrigger("elevatorFloor").onTrue(claw.rotateWrist(PositionState.Floor))
                                                                .onTrue(elevatorMotor1.setDesiredHeight(PositionState.Floor))
                                                                .onTrue(elevatorMotor2.setDesiredHeight(PositionState.Floor));
        new EventTrigger("elevatorBarge").onTrue(claw.rotateWrist(PositionState.Barge))
                                                                .onTrue(elevatorMotor1.setDesiredHeight(PositionState.Barge))
                                                                .onTrue(elevatorMotor2.setDesiredHeight(PositionState.Barge));
        //new EventTrigger("elevatorL2").onTrue(Commands.runOnce(()->{setAutoHeight(PositionState.L2);}));
        //new EventTrigger("outtakeAlgae").onTrue(Commands.runOnce(()->{System.out.println("dont");}));
        
        //new EventTrigger("outtakeAlgae").whileTrue(Commands.runOnce(()->{SmartDashboard.autoWorking = true;}));
        //autoChooser = AutoBuilder.buildAutoChooser("New New New Auto");
       
        autoChooser = AutoBuilder.buildAutoChooser("L4 and Processor");
        /*Adds autos to autochooser to show on Smart Dashboard 
         * name is what you want it to show on dashboard. autoname is the pathplanner name */
        autoChooser.addOption("Test Named Command", AutoBuilder.buildAuto("Test Named Command"));
        //autoChooser.addOption("New New New", AutoBuilder.buildAuto("New New New Auto"));
        autoChooser.addOption("Forward Blue 1M", AutoBuilder.buildAuto("Forward Blue 1M")); 
        
        SmartDashboard.putData("Auto Chooser", autoChooser);
    
        SmartDashboard.putBoolean("autoWorking", false);

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
        {
            s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                    s_Swerve, 
                    () -> -m_driverController.getLeftY(),
                    () -> -m_driverController.getLeftX(), 
                    () -> -m_driverController.getRightX(), 
                    () -> true
                )
            );
        }
        else if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
        {
            s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                    s_Swerve, 
                    () -> m_driverController.getLeftY(),
                    () -> m_driverController.getLeftX(), 
                    () -> m_driverController.getRightX(), 
                    () -> true
                )
            );
        }
        
        // frontCamera = CameraServer.startAutomaticCapture("camera", 0);
        //CameraServer.startAutomaticCapture("camera", 0);
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
        setAxisAction(ButtonMap.Barge, PositionState.Barge);
        setAxisAction(ButtonMap.Floor, PositionState.Floor);
        setAxisAction(ButtonMap.Processor, PositionState.Processor);

        m_operatorController.button(ButtonMap.cIn).onTrue(intake.Intake_coral());
        m_operatorController.button(ButtonMap.cOut).whileTrue(intake.Outtake());

        
        m_driverController.button(ButtonMap.aIn).whileTrue(intake.Intake_Algea());
        m_driverController.button(ButtonMap.aOut).whileTrue(intake.Outtake_Algea());
        m_driverController.button(ButtonMap.aOut).onTrue(claw.flickWrist());

        m_driverController.button(4).onTrue(s_Swerve.invertControlsCommand());

        m_driverController.button(8).onTrue(s_Swerve.resetModulesToAbsolute());
    }

    private void setButtonAction(int button, PositionState state)
    {
        m_operatorController.button(button).onTrue(claw.rotateWrist(state));
        m_operatorController.button(button).onTrue(elevatorMotor1.setDesiredHeight(state));
        m_operatorController.button(button).onTrue(elevatorMotor2.setDesiredHeight(state));
    }
    private void setAutoHeight( PositionState state)
    {
        System.out.println("test elevator");
        claw.rotateWrist(state);
        elevatorMotor1.setDesiredHeight(state);
        elevatorMotor2.setDesiredHeight(state);
    }
    

    private void setAxisAction(int button, PositionState state)
    {
        m_operatorController.axisMagnitudeGreaterThan(button, 0).onTrue(claw.rotateWrist(state));
        m_operatorController.axisMagnitudeGreaterThan(button, 0).onTrue(elevatorMotor1.setDesiredHeight(state));
        m_operatorController.axisMagnitudeGreaterThan(button,0).onTrue(elevatorMotor2.setDesiredHeight(state));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
        
    }
}