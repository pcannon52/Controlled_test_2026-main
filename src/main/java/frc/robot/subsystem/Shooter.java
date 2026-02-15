// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BotConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private static Shooter Shooter = null;

  //Prevents the need of duplicate objects
  public static synchronized Shooter get(){
    if(Shooter == null){
      Shooter = new Shooter();
    } 
    return Shooter;
  }

  public static enum State{
    IDLE(0,0),
    OUTPUT(10, BotConstants.Shooter.velocityTable.get(Hood.get().getHoodAngleDegrees())); // the 1 will be replaced with hood angle

    double inputSpeed;
    double outputSpeed;

    State(double intputSpeed, double  ouptutSpeed){
      this.inputSpeed = intputSpeed;
      this.outputSpeed = ouptutSpeed;
    }
  }

   //Motors
  private final TalonFX mShooterOutput = new TalonFX(BotConstants.Shooter.shooterInputID);
  private final TalonFX mShooterInput = new TalonFX(BotConstants.Shooter.shooterOutputID);
  // private final TalonFX mShooterRoller = new TalonFX(BotConstants.Intake.intakeID); // this is will be its own subsystem

    private final MotionMagicVelocityVoltage InputVelocityController  = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVelocityVoltage OutputVelocityController  = new MotionMagicVelocityVoltage(0);

StatusSignal<AngularVelocity> velocitySignal;

  public Shooter(){
    mShooterOutput.getConfigurator().apply(BotConstants.Shooter.cfg_shooter_Output);
    mShooterInput.getConfigurator().apply(BotConstants.Shooter.cfg_shooter_Input);
    
    this.velocitySignal = mShooterOutput.getVelocity();

    this.setDefaultCommand(doIdle());
  }
  
public Command doIdle() {
    return run(() -> {
        mShooterInput.setControl(InputVelocityController.withVelocity(State.IDLE.inputSpeed));
        mShooterOutput.setControl(OutputVelocityController.withVelocity((State.IDLE.outputSpeed)));
    });
}

public Command runShooter(){
  return run(
    ()->{ 
     mShooterOutput.setControl(OutputVelocityController.withVelocity((State.OUTPUT.outputSpeed)));
    if(atDesiredSpeed()){
    mShooterInput.setControl(InputVelocityController.withVelocity(State.OUTPUT.inputSpeed));
    }
    });
}

public boolean atDesiredSpeed(){
return Math.abs(getShooterVelocity() - State.OUTPUT.outputSpeed) <= 0.3; // clerance of .3, chosen arbitrarily, must test if it works
}

public double getShooterVelocity(){
return velocitySignal.refresh().getValueAsDouble();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Outputting velocity", this.getShooterVelocity());
  }
}
