package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Common.Coordinates;
import org.firstinspires.ftc.teamcode.Common.RobotTeamColor;
@Autonomous

public class RightRedInit extends OpMode
{
    AutonomousEntryPoint autonomousEntryPoint;

    @Override
    public void init() {
        autonomousEntryPoint = new AutonomousEntryPoint(
                new Coordinates(FieldModel.CM_PER_TILE*4.5, 37/2), RobotTeamColor.Red, hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        autonomousEntryPoint.loop();
    }
}

