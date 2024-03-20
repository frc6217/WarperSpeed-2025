// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



/*
 * Pipelines
 * 
 * AprilTag Camera
 * 
 * 0 - Red Speaker
 * 1 - Blue Speaker
 * 2 - Red Amp
 * 3 - Blue Amp
 * 4 - Red Trap
 * 5 - Blue Trap
 * 6 - Red Source
 * 7 - Blue Source
 * 
 */
public class LimeLightSub extends SubsystemBase {
  /** Creates a new limelight. */
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tid;
  NetworkTableEntry tv;
  NetworkTableEntry targetSkew;
  String name;

  public LimeLightSub(String name) {
    table = NetworkTableInstance.getDefault().getTable(name);
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tid = table.getEntry("tid");
    tv = table.getEntry("tv");

    this.name = name;
    for(int port = 5800; port <= 5807; port++){
      PortForwarder.add(port, name + ".local", port);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tid = table.getEntry("tid");
    tv = table.getEntry("tv");

    targetSkew = table.getEntry("ts");
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber(name + " X", x);
    SmartDashboard.putNumber(name + " Y", y);
    SmartDashboard.putNumber(name + " Area", area);
    SmartDashboard.putNumber(name + " skew", targetSkew.getDouble(0));
    SmartDashboard.putBoolean(name + " seetarget", (tv.getDouble(0)) != 0);
  }

  public double getArea(){
    return ta.getDouble(0);
  }
  public double getX(){
    return tx.getDouble(0);
  }
  public double getY(){
    return ty.getDouble(0);
  }

  public AprilTag getAprilTag() {
    // return april tag of tid
    return new AprilTag((int)tid.getInteger(0), new Pose3d());
  }

  public boolean isValid() {
    // check and returns tv
    return (tv.getDouble(0) > .5);
  }

  public double getSkew(){
    return targetSkew.getDouble(0);
  }

  public void setPipeline(int id) {
    NetworkTableInstance.getDefault().getTable(name).getEntry("pipeline").setNumber(id);

  }
}
