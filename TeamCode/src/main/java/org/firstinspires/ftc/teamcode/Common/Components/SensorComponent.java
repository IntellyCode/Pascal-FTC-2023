package org.firstinspires.ftc.teamcode.Common.Components;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SensorComponent {
    public BNO055IMU imu;
    public DistanceSensor distanceSensor;

    public SensorComponent(HardwareMap hardwareMap){
        this.imu = hardwareMap.get(BNO055IMU.class, "gyro");
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, "dist");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "Calibration.json";
        imu.initialize(parameters);
    }

}
