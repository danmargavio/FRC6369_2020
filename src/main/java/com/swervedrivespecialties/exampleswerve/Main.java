//package frc.robot;
package com.swervedrivespecialties.exampleswerve;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;

public final class Main {
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
