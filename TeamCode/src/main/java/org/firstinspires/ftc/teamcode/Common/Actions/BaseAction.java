package org.firstinspires.ftc.teamcode.Common.Actions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.RobotModel;

public abstract class BaseAction implements IAction {
    protected RobotModel robotModel;
    Telemetry telemetry;
    public BaseAction(RobotModel robotModel, Telemetry telemetry) {
        this.robotModel = robotModel;
        this.telemetry = telemetry;
    }
    protected boolean finished;
    @Override
    public boolean isFinished() {
        return finished;
    }
}
