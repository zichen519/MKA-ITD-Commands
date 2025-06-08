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
    public static double feedforwardConstant = 0.0;

    public static double ticksPerRevolution = 1993.6;  // GoBuilda Yellow Jacket 84 RPM
    public static double gearRatio = 2.5;
    // PID values - keep at zero for pure feedforward tuning
    public static double p = 0.0;
    public static double i = 0.0;
    public static double d = 0.0;

    // Enable/disable the feedforward
    public static boolean enableFeedforward = true;

    // Motors
    public MotorEx liftMotor;

    private double calculateFeedforward() {
        return Math.cos(Math.toRadians(target / ((ticksPerRevolution * gearRatio)/360))) * 0.01;
    }
    // Controller
    public PIDFController controller;

    // Motor names
    public String liftMotorName = "lift";


    // Current target (will be set to current position)
    private double target = 0;

    @Override
    public void onInit() {
        // Initialize motors

        liftMotor = new MotorEx(liftMotorName);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset encoders
        liftMotor.resetEncoder();


        // Create motor group


        // Initialize controller with zero feedforward


        // Arm feedforward (better for pivoting mechanisms than static feedforward)

        controller = new PIDFController(p, i,d, v -> calculateFeedforward());
        controller.setSetPointTolerance(30);

        // Set target to current position
        target = liftMotor.getCurrentPosition();

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
        liftMotor.setPower(controller.calculate(liftMotor.getCurrentPosition(), target));
        // Update feedforward if it changed


        // Update target to current position (so PID doesn't fight manual movement)


        // Apply control


        // Telemetry
        telemetry.addLine("=== FEEDFORWARD TUNING ===");
        telemetry.addData("Current Position", "%.1f", liftMotor.getCurrentPosition());


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

        telemetry.addData("Left Power", "%.3f", liftMotor.getPower());


        telemetry.update();
    }
}