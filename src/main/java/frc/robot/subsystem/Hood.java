// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BotConstants;

public class Hood extends SubsystemBase {
private static Hood hood = null;

  //Prevents the need of duplicate objects
  public static synchronized Hood get(){
    if(hood == null){
      hood = new Hood();
    } 
    return hood;
  }

  public enum State{
    IDLE(15.0),
    READY_TO_SHOOT(BotConstants.Hood.shooterTable.get(Swerve.get().distanceToHub()));

    double hoodAngle;

    private State(double hoodAngle){
      this.hoodAngle = hoodAngle;
  }
  }


  private final TalonFX mHoodAngle = new TalonFX(BotConstants.Hood.Hood_ID);
  private final MotionMagicVoltage hoodAngleController = new MotionMagicVoltage(0);
  private StatusSignal<Angle> motorPosition;
  

  public Hood(){
  mHoodAngle.getConfigurator().apply(BotConstants.Hood.cfg_Hood);
  this.motorPosition = mHoodAngle.getPosition();
  }

     public double getHoodAngleDegrees() {
        double motorRotations = motorPosition.refresh().getValueAsDouble(); 
        double hoodRotations = motorRotations / 3; // 3 is just a fake gear ratio
        return hoodRotations * 360.0;
    }

    public Command runHood(){
    return run(
    ()->{mHoodAngle.setControl(hoodAngleController.withPosition(State.READY_TO_SHOOT.hoodAngle));}
    );
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Angle", this.getHoodAngleDegrees());
  }
}