// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tid = table.getEntry("tid");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  public Limelight() {

  }

  public void SetPipeline(int pipeline){
    NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
    pipelineEntry.setNumber(pipeline);
  }

  public double ReefTagX(){
    return tx.getDouble(0.0);
  }

  public double ReefTagY(){
    return tx.getDouble(0.0);
  }

  public double ReefTagArea(){
    return ta.getDouble(0);
  }

  public double ReefTagID(){
    return tid.getDouble(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  SmartDashboard.putNumber("LimeLight X Value", tx.getDouble(0.0));
  SmartDashboard.putNumber("LimeLight Y Value", ty.getDouble(0.0));
  SmartDashboard.putNumber("LimeLight Target Area", ta.getDouble(0.0));
  SmartDashboard.putNumber("Reef AprilTag ID", tid.getDouble(0.0));

  }
}
