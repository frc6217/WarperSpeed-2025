// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSub extends SubsystemBase {
  /** Creates a new limelight. */
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  String name;

  public LimeLightSub(String name) {
    table = NetworkTableInstance.getDefault().getTable(name);
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
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
    NetworkTableEntry targetSkew = table.getEntry("ts");
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber(name + " X", x);
    SmartDashboard.putNumber(name + " Y", y);
    SmartDashboard.putNumber(name + " Area", area);
    SmartDashboard.putNumber(name + " skew", targetSkew.getDouble(0));
    SmartDashboard.putBoolean(name + " seetarget", (area != 0));
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
}
