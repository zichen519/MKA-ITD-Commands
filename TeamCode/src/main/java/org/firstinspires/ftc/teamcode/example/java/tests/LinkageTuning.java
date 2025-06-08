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

@TeleOp(name = "Linkage Tuning")
@Config
public class LinkageTuning extends NextFTCOpMode {

    // Dashboard configurable values
    public static double f = 0.5;  // Cosine feedforward constant
    public static double p = 0.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static int target = 0;


    // Linkage mechanical parameters
    public static double ticksPerRevolution = 1993.6;  // GoBuilda Yellow Jacket 84 RPM
    public static double gearRatio = 2.5;  // 24-tooth to 60-tooth gear (60/24 = 2.5:1)

    // Test positions (these will be different now due to gearing)
      // ~108 degrees

    // Motor
    public MotorEx linkageMotor;

    // Controller



    // Motor name
    public String motorName = "lift";

    // Custom feedforward calculation
    private double calculateFeedforward() {
        return Math.cos(Math.toRadians(target / ((ticksPerRevolution * gearRatio)/360))) * f;
    }
    public PIDFController controller = new PIDFController(p, i, d, v -> calculateFeedforward());
    @Override
    public void onInit() {
        // Initialize motor
        linkageMotor = new MotorEx(motorName);
        linkageMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        linkageMotor.resetEncoder();

        // Initialize controller
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller.setSetPointTolerance(30);

        // Set target to current position
        target = (int)linkageMotor.getCurrentPosition();



    }



    @Override
    public void onUpdate() {
        // Update controller with dashboard values
        controller.setKP(p);
        controller.setKI(i);
        controller.setKD(d);
        linkageMotor.setPower(controller.calculate(linkageMotor.getCurrentPosition(), target));
        // Calculate custom feedforward








        // Calculate angles for display
        double currentAngle = linkageMotor.getCurrentPosition() / (ticksPerRevolution * gearRatio) * 360;
        double targetAngle = target / (ticksPerRevolution * gearRatio) * 360;


        // Telemetry
        telemetry.addLine("=== CUSTOM FEEDFORWARD TUNING ===");
        telemetry.addData("Current Position", "%.1f ticks", linkageMotor.getCurrentPosition());
        telemetry.addData("Target Position", "%d ticks", target);
        telemetry.addData("Current Angle", "%.1f°", currentAngle);
        telemetry.addData("Target Angle", "%.1f°", targetAngle);
        telemetry.addData("Motor Power", "%.3f", linkageMotor.getPower());
        telemetry.addData("ff", calculateFeedforward());
        telemetry.addLine();




        telemetry.update();
    }


}