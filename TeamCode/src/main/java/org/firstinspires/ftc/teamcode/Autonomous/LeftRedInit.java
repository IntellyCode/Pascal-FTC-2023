package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Common.Coordinates;
import org.firstinspires.ftc.teamcode.Common.RobotTeamColor;
@Autonomous

public class LeftRedInit extends OpMode {
    AutonomousEntryPoint autonomousEntryPoint;

    @Override
    public void init() {
        autonomousEntryPoint = new AutonomousEntryPoint(
                new Coordinates(FieldModel.CM_PER_TILE*1.5, 37/2), RobotTeamColor.Red, hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        autonomousEntryPoint.loop();
    }
}
