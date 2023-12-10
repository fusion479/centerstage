package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends SubsystemBase {
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    private final PIDFController controller = new PIDFController(0.0025,0,0, 0 );
    private double target;

    public void configureMotors() {
        this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // might be wrong RunMode
        this.rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        this.leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.leftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        this.rightMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public Lift(final HardwareMap hwMap) {
        this.leftMotor = hwMap.get(DcMotorEx.class, "liftLeft");
        this.rightMotor = hwMap.get(DcMotorEx.class, "liftRight");

        this.configureMotors();
    }

    @Override
    // run once per scheduler run
    public void periodic() {
        double velocity = this.controller.calculate(this.leftMotor.getCurrentPosition() + 1.03, target); // 0.03 is Kg
        this.leftMotor.setVelocity(velocity);
        this.rightMotor.setVelocity(velocity);
    }

    public void setTarget(double target) {
        this.target = target;
    }
}
