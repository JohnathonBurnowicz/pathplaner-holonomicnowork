// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public Limelight() {

   /**   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0); 
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area); */

    //float std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
   
    //float tv = table->GetNumber("tv");
    //float tx = table->GetNumber("tx");

    //float steering_adjust = 0.0f;
    //if (tv == 0.0f)
    {
        // We don't see the target, seek for the target by spinning in place at a safe speed.
       // steering_adjust = 0.3f;     
    }
    //else
    {
        // We do see the target, execute aiming code
       // float heading_error = tx;
      //      steering_adjust = Kp * tx;
    }
            
    //left_command+=steering_adjust;
    //right_command-=steering_adjust;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
