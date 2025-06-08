package org.firstinspires.ftc.teamcode.example.java.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;

@TeleOp(name = "Feedforward Tuning")
@Config
public class FFTuning extends NextFTCOpMode {

    public FFTuning() {
        super();
    }

    // Dashboard configurable feedforward constant
    public static double feedforwardConstant = 0.09;

    // PID values - keep at zero for pure feedforward tuning
    public static double p = 0.0;
    public static double i = 0.0;
    public static double d = 0.0;

    // Enable/disable the feedforward
    public static boolean enableFeedforward = true;

    // Motors
    public MotorEx leftMotor;
    public MotorEx rightMotor;
    public MotorGroup liftMotors;

    // Controller
    public PIDFController controller;

    // Motor names
    public String leftMotorName = "slide1";
    public String rightMotorName = "slide2";

    // Current target (will be set to current position)
    private double target = 0;

    @Override
    public void onInit() {
        // Initialize motors
        leftMotor = new MotorEx(leftMotorName);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor = new MotorEx(rightMotorName);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset encoders
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();

        // Create motor group
        liftMotors = new MotorGroup(leftMotor, rightMotor);

        // Initialize controller with zero feedforward
        controller = new PIDFController(0.0, 0.0, 0.0, new StaticFeedforward(feedforwardConstant));
        controller.setSetPointTolerance(30);

        // Set target to current position
        target = liftMotors.getCurrentPosition();

        telemetry.addLine("=== FEEDFORWARD TUNING MODE ===");
        telemetry.addLine("1. Lift slides by hand to desired height");
        telemetry.addLine("2. Increase feedforwardConstant in dashboard");
        telemetry.addLine("3. Find value where slides don't fall");
        telemetry.addLine("4. That's your feedforward value!");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Set target to current position when started


        telemetry.addLine("=== TUNING STARTED ===");
        telemetry.addLine("Manually lift slides and tune feedforward!");
        telemetry.update();
    }

    public void onUpdate() {
        // Update controller with dashboard values
        controller.setKP(p);
        controller.setKI(i);
        controller.setKD(d);
        liftMotors.setPower(controller.calculate(liftMotors.getCurrentPosition(), target));
        // Update feedforward if it changed


        // Update target to current position (so PID doesn't fight manual movement)


        // Apply control


        // Telemetry
        telemetry.addLine("=== FEEDFORWARD TUNING ===");
        telemetry.addData("Current Position", "%.1f", liftMotors.getCurrentPosition());


        telemetry.addLine("=== DASHBOARD VALUES ===");
        telemetry.addData("Feedforward Constant", "%.4f", feedforwardConstant);


        telemetry.addLine();



        telemetry.addLine("=== TUNING TIPS ===");
        telemetry.addLine("• Start with 0.01, increase by 0.01");
        telemetry.addLine("• Typical values: 0.03 - 0.15");
        telemetry.addLine("• Too high = slides drift up");
        telemetry.addLine("• Too low = slides fall down");
        telemetry.addLine("• Just right = minimal drift");

        // Motor status
        telemetry.addLine();
        telemetry.addLine("=== MOTOR STATUS ===");
        telemetry.addData("Left Motor", "%.1f", leftMotor.getCurrentPosition());
        telemetry.addData("Right Motor", "%.1f", rightMotor.getCurrentPosition());
        telemetry.addData("Left Power", "%.3f", leftMotor.getPower());
        telemetry.addData("Right Power", "%.3f", rightMotor.getPower());

        telemetry.update();
    }
}