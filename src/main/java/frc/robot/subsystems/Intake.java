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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Constants.IntakeConstants.*;

//Import required WPILib libraries
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;

public class Intake extends SubsystemBase {
  /** Creates a new Shooter. */
  private SparkMax motor;
  private RelativeEncoder encoder;
  private SparkClosedLoopController pid;
  private double desiredVelocity = 0;
  //Create an instance of the AnalogInput class so we can read from it later
  private DigitalInput photoelctricSensor;  // TODO: Check channel

  boolean intakeCoral = false;
  

  public Intake() {
    SmartDashboard.putData("call it", Outtake_Algea());
    motor = new SparkMax(kCANID, SparkMax.MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    config.encoder
        .velocityConversionFactor(1);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(0, 0, 0, 0.000175);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();

    photoelctricSensor = new DigitalInput(0);

  }

  public boolean getPhotoSensor() {
    return !photoelctricSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Intake", encoder.getVelocity());
    SmartDashboard.putNumber("Desired Intake", desiredVelocity);
    SmartDashboard.putBoolean("Photo Sensor", getPhotoSensor());
    pid.setReference(desiredVelocity, ControlType.kVelocity);

    // Check sensor
    if (intakeCoral && desiredVelocity != 0 && getPhotoSensor())
    {
        desiredVelocity = 0;
        intakeCoral = false;
        
    }
  }
 
  public Command Intake_coral() {
    return runOnce(() -> {
      desiredVelocity = -2000;
      intakeCoral = true;
    });
  }

  public Command Outtake() {
    return runEnd(() -> {
      desiredVelocity = -2000;
    }, () -> {
      desiredVelocity = 0;
    });
  }
  public Command OuttakeStop() {
    return runEnd(() -> {
      desiredVelocity = 0;
    }, () -> {
      desiredVelocity = 0;
    });
  }

  public Command Intake_Algea() {
    return runEnd(() -> {
      desiredVelocity = 4000;
    }, () -> {
      desiredVelocity = 1000;
    });
  }

  public Command Outtake_Algea() {
    return runEnd(() -> {
      desiredVelocity = -6000;
    }, () -> {
      desiredVelocity = 0;
    });
  }
  

}
