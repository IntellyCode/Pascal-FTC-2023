package org.firstinspires.ftc.teamcode.Common.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.RobotModel;
import org.firstinspires.ftc.teamcode.Common.Components.ArmComponent;

public class LiftArmAction extends BaseAction {
    ArmComponent armComponent;
    double position;
    int targetTicks;

    public LiftArmAction(RobotModel robotModel, double position, Telemetry telemetry) {
        super(robotModel, telemetry);
        this.position = position;
        armComponent = robotModel.getArmComponent();
    }

    @Override
    public void start() {
        if(!finished) {
            if(position > armComponent.MAXIMUM_ARM_POSITION)
                position = armComponent.MAXIMUM_ARM_POSITION;

            targetTicks = (int)(armComponent.ARM_TICKS_PER_CM * position);
            armComponent.armMotor.setTargetPosition(targetTicks);
            armComponent.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    @Override
    public void update() {
        if(!finished && armComponent.armMotor.getCurrentPosition() == targetTicks) {
            robotModel.armPosition = targetTicks / armComponent.ARM_TICKS_PER_CM;
            finished = true;
        }
    }

}
