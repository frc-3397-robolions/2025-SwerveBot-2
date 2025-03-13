// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States;
import frc.robot.Constants.States.PositionState;
import frc.robot.utilities.MathUtils;

import static frc.robot.Constants.ClawConstants.*;


//import org.apache.commons.collections4.map.HashedMap;
public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private final SparkMax angleMotor;
  private final RelativeEncoder angleEncoder;
  private final SparkClosedLoopController anglePID;
  private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(kMaxVel, kMaxAccel));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private boolean intakeDesiredOut = false;
  private boolean intakeArrived = false;
  private boolean intaking = false;
  private boolean outtaking = false;
  private double currentPosition = 0.0;
  private PositionState currentState = PositionState.Home;
  private Elevator elevator;
  private boolean at12Deg = false;

  private boolean L1Involved = true;

  public Claw(Elevator elevatorIn) {
    elevator = elevatorIn;
    // The motor controlling the angle of the assembly
    angleMotor = new SparkMax(CANIDAngle, SparkMax.MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    config.encoder
      //.positionConversionFactor(2*Math.PI)
      //.velocityConversionFactor(2*Math.PI);
      // TODO
        .positionConversionFactor(kAnglePositionFactor)
        .velocityConversionFactor(kAnglePositionFactor);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kP, kI, kD, kFF);
        //.positionWrappingEnabled(true)
        //.positionWrappingInputRange(0, 2*Math.PI);

    angleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // angleMotor.setSmartCurrentLimit(CurrentLimit.kIntakeAngle);
    // angleMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
    
    // angleMotor.setSoftLimit(SoftLimitDirection.kForward, intakeStates.get(true).floatValue());
    // angleMotor.setSoftLimit(SoftLimitDirection.kReverse, intakeStates.get(false).floatValue());

    angleEncoder = angleMotor.getEncoder();
    anglePID = angleMotor.getClosedLoopController();
    
    // The motor driving the intake wheels
    //driveMotor = new SparkMax(CANIDDrive, SparkMax.MotorType.kBrushless);
    //SparkMaxConfig configDrive = new SparkMaxConfig();
    //configDrive
    //    .idleMode(IdleMode.kBrake);
    //driveMotor.setSmartCurrentLimit(CurrentLimit.kIntakeWheels); // TODO: do we need this
    //driveMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
    
  }

  @Override
  public void periodic() {

    // If the intake is within tolerance of it's desired angle, set this variable to
    // true so other files can act

    SmartDashboard.putBoolean("Elevator Arrived", elevator.ElevatorArrived);
    SmartDashboard.putBoolean("moving up", elevator.movingUP);
    if(L1Involved && !elevator.ElevatorArrived && elevator.movingUP && (elevator.getHeight() > -7))
    {
      goToWristPosition(-1 * Math.toRadians(30));
    } 
    else if (L1Involved && !elevator.ElevatorArrived && !elevator.movingUP && (elevator.getHeight() < -4))
    {
      goToWristPosition(-1 * Math.toRadians(30));
    } 
    else
    {
      goToWristPosition(currentPosition);
    }

    if (MathUtils.angleInRange(angleEncoder.getPosition(), currentPosition, kPositionTolerance))
      intakeArrived = true;
    else
      intakeArrived = false;

    // Drive intake wheels based on the desired state

    /*
    if (intaking)
      driveMotor.set(kIntakePower);
    else if (outtaking)
      driveMotor.set(kOuttakePower);
    else
      driveMotor.set(0);
    */
    // Putting data on SmartDashboard
    SmartDashboard.putNumber("Wrist Angle", Math.toDegrees(getWristPosition()));
    SmartDashboard.putBoolean("Intaking", intaking);
    SmartDashboard.putBoolean("Outtaking", outtaking);
    SmartDashboard.putNumber("Intake Power", angleMotor.getAppliedOutput());
  }

  public boolean getIntakeArrived() {
    return intakeArrived;
  }

  public Command zeroIntake() {
    return runOnce(() -> {
      angleEncoder.setPosition(0);
    });
  }

  public double getWristPosition() {
    return angleEncoder.getPosition() ;
  }

  public Command rotateWrist(States.PositionState state) {
    return runOnce(() -> {
      L1Involved = currentState == PositionState.Home || state == PositionState.Home;
      currentPosition = -1 * Math.toRadians(States.DesiredAngleMap.get(state).floatValue());
      SmartDashboard.putNumber("Wrist Desired Angle", States.DesiredAngleMap.get(state).floatValue());
      currentState = state;
    });
  }

  public Command flickWrist() {
    return runOnce(() -> {
      if (currentState == States.PositionState.Barge){
        L1Involved = false;
        currentPosition += (Math.toRadians(45.0));
        SmartDashboard.putNumber("Wrist Desired Angle", currentPosition);
      }
    });
  }
  public void goToWristPosition(double goal_position) {
    m_goal = new TrapezoidProfile.State(goal_position, 0);
    m_setpoint = profile.calculate(2, m_setpoint, m_goal);
    // Set the PID Controller to the corresponding angle for in/out
    anglePID.setReference(m_setpoint.position, ControlType.kPosition);
  }
  
}
