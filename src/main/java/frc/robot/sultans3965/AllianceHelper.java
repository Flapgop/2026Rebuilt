package frc.robot.sultans3965;

import edu.wpi.first.wpilibj.DriverStation;

public class AllianceHelper {
    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
    }

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }
}
