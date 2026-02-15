// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BotConstants;



public class Hopper extends SubsystemBase {

  
  public static Hopper hopper = null;


  public static synchronized Hopper get(){
    if(hopper == null){
      hopper = new Hopper();
    }
    return hopper;
  }

  public enum State{
    OFF(0),
    ON(1);

    double rollerSpeed;

    private State(double rollerSpeed){
      this.rollerSpeed = rollerSpeed;
  }
  }

  //Motor controller
  private static MotionMagicVelocityVoltage HopperController = new MotionMagicVelocityVoltage(0);
  //Motor initalized
  private final TalonFX mHopper = new TalonFX(BotConstants.Hopper.HopperID);
  
  private State mHopperState = State.OFF;

  public Hopper() {
    //Motor config
    mHopper.getConfigurator().apply(BotConstants.cfg_Roller);
    this.setDefaultCommand(runHopper());

  }

  //Moves the belt that pushes all the balls towrads the shooter
  public Command runHopper(){
    return run(()->
    {
     if(isBroken()){
      mHopper.setControl(HopperController.withVelocity(State.ON.rollerSpeed));
      mHopperState = State.ON;}
    });
  }

  public boolean isBroken(){ // always true for now
    return true;
  }

  //Stops
  public Command Stop(){
    return run(()->{
      mHopper.setControl(HopperController.withVelocity(State.OFF.rollerSpeed));
      mHopperState = State.OFF;
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("State of hopper", mHopperState.toString());
  }
}