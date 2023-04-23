package org.firstinspires.ftc.teamcode.Common.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class ArmComponent {
    public final double ARM_WHEEL_DIAMETER = 5.75;
    public final double TICKS_PER_REVOLUTION = 1120;
    public final int ARM_TICKS_PER_CM = (int)(TICKS_PER_REVOLUTION/(ARM_WHEEL_DIAMETER * Math.PI));
    public final double GRAB_POSITION_CM = 0.1;
    public static final double PALKA_1_CM = 34.29;
    public static final double PALKA_2_CM = 58.42;
    public static final double PALKA_3_CM = 89;

    //Сюда вставить ограничение на подъём руки.
    public final double MAXIMUM_ARM_POSITION = 90;
    public DcMotor armMotor;
    public Servo rightClaw;
    public Servo leftClaw;

    //public int armMotorPosition;
    public ArmComponent(HardwareMap hardwareMap) {
        this.armMotor = hardwareMap.get(DcMotor.class, "lift");
        this.rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        this.leftClaw = hardwareMap.get(Servo.class, "leftClaw");

        rightClaw.getController().pwmEnable();
        rightClaw.setPosition(0.5);
        rightClaw.setDirection(Servo.Direction.FORWARD);
        leftClaw.getController().pwmEnable();
        leftClaw.setPosition(0);
        leftClaw.setDirection(Servo.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setPower(1);
    }
}
