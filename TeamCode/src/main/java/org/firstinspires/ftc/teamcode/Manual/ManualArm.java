package org.firstinspires.ftc.teamcode.Manual;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Common.Components.ArmComponent;
import org.firstinspires.ftc.teamcode.Common.Components.ArmPosition;
import org.firstinspires.ftc.teamcode.Common.RobotModel;

public class ManualArm {
    RobotModel model;
    ArmComponent armComponent;


    public ManualArm(RobotModel model) {
        this.model = model;
        this.armComponent = model.getArmComponent();
    }

    public void setCleshnja(boolean open) {
        if (open) {
            model.withCone = true;
            armComponent.rightClaw.setPosition(0);
            armComponent.leftClaw.setPosition(0.5);
        } else {
            model.withCone = false;
            armComponent.rightClaw.setPosition(0.5);
            armComponent.leftClaw.setPosition(0);
        }
    }
    public void liftArmToPosition(ArmPosition position) {

        switch (position) {
            case Zero:
                armComponent.armMotor.setTargetPosition(-(int) (armComponent.ARM_TICKS_PER_CM * armComponent.GRAB_POSITION_CM));

            case First:
                armComponent.armMotor.setTargetPosition((int) (armComponent.ARM_TICKS_PER_CM * armComponent.PALKA_1_CM));

            case Second:
                armComponent.armMotor.setTargetPosition((int) (armComponent.ARM_TICKS_PER_CM * armComponent.PALKA_2_CM));

            case Third:
                armComponent.armMotor.setTargetPosition((int) (armComponent.ARM_TICKS_PER_CM * armComponent.PALKA_3_CM));

            }

            armComponent.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
    public void controlLiftManually(int power){
        armComponent.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armComponent.armMotor.setPower(power);
    }
}



