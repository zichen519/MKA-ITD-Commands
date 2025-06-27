package org.firstinspires.ftc.teamcode.example.java.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;

@TeleOp(name = "Lift Tuning")
@Config
public class LiftTuning extends NextFTCOpMode {

    // Dashboard configurable values
 // Cosine feedforward constant
    public static double p = 0.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0.14;
    public static int target = 0;


    // Linkage mechanical parameters
      // 24-tooth to 60-tooth gear (60/24 = 2.5:1)

    // Test positions (these will be different now due to gearing)
    // ~108 degrees

    // Motor
    public MotorEx leftMotor;
    public MotorEx rightMotor;

    // Controller



    // Motor name
    public String leftMotorName = "slide1";
    public String rightMotorName = "slide2";
    public MotorGroup liftMotors;

    // Custom feedforward calculation

    public PIDFController controller = new PIDFController(p, i, d, v -> f, 30);
    @Override
    public void onInit() {
        // Initialize motor
        leftMotor = new MotorEx(leftMotorName);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor = new MotorEx(rightMotorName);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
        liftMotors = new MotorGroup(leftMotor, rightMotor);

        // Initialize controller
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        // Set target to current position
        target = (int)leftMotor.getCurrentPosition();



    }



    @Override
    public void onUpdate() {
        // Update controller with dashboard values
        controller.setKP(p);
        controller.setKI(i);
        controller.setKD(d);
        liftMotors.setPower(controller.calculate(liftMotors.getCurrentPosition(), target));
        // Calculate custom feedforward








        // Calculate angles for display



        // Telemetry
        telemetry.addLine("=== CUSTOM FEEDFORWARD TUNING ===");
        telemetry.addData("Current Position", "%.1f ticks", leftMotor.getCurrentPosition());
        telemetry.addData("Target Position", "%d ticks", target);

        telemetry.addData("Motor Power", "%.3f", leftMotor.getPower());

        telemetry.addLine();




        telemetry.update();
    }


}