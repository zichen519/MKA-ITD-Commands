package org.firstinspires.ftc.teamcode.example.java.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;
@Config
public class Lift extends Subsystem {
    // BOILERPLATE
    public static final Lift INSTANCE = new Lift();
    private Lift() { }

    // USER CODE
    public MotorEx leftMotor;
    public MotorEx rightMotor;
    public MotorGroup liftMotors;
    public static double p = 0.0125;
    public static double i = 0.0;
    public static double d = 0.00025;
    public static double f = 0.14;
    //public static int target = 0;
    public PIDFController controller = new PIDFController(0.02, 0.0, 0.0004, v -> f,30);




    public String leftMotorName = "slide1";
    public String rightMotorName = "slide2";

    public Command manualControl(float power) {
        if(!Linkage.INSTANCE.linkageDown){
            return new SetPower(liftMotors, (double)power, this);
        }
        else{
            if(leftMotor.getCurrentPosition()<350  || power < 0){
                return new SetPower(liftMotors, (double)power, this);
            }
            else return new SetPower(liftMotors, 0,this);
        }

    }


    public Command retract() {
        return new RunToPosition(liftMotors, // MOTOR TO MOVE
                0.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toLowBasket() {
        return new RunToPosition(liftMotors, // MOTOR TO MOVE
                0.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toHighBasket() {
        //OpModeData.telemetry.addLine("toHighBasket() called!"); // Debug line
        OpModeData.telemetry.update();
        return new RunToPosition(liftMotors, // MOTOR TO MOVE
                900, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toHighChamber() {
        return new RunToPosition(liftMotors, // MOTOR TO MOVE
                600, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command setTarget(int newTarget) {

        return new RunToPosition(liftMotors, (double)newTarget, controller, this);
    }
    @Override
    public Command getDefaultCommand(){
        return new HoldPosition(liftMotors,controller,this);
    }

    @Override
    public void initialize() {
        leftMotor = new MotorEx(leftMotorName);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor = new MotorEx(rightMotorName);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
        liftMotors = new MotorGroup(leftMotor, rightMotor);


        

    }



    public void periodic() {
        // This method can be used for periodic updates if needed
        /*
        controller.setKP(p);
        controller.setKI(i);
        controller.setKD(d);
        liftMotors.setPower(controller.calculate(liftMotors.getCurrentPosition(), target));
        */

        OpModeData.telemetry.addData("Lift Position", liftMotors.getCurrentPosition());

        /*
        OpModeData.telemetry.addData("Lift Target", target);
        OpModeData.telemetry.addData("FConstant", f);
        OpModeData.telemetry.addData("PConstant", controller.getKP());
        OpModeData.telemetry.addData("IConstant", controller.getKI());
        OpModeData.telemetry.addData("DConstant", controller.getKD());
        OpModeData.telemetry.addLine("=== MOTOR STATUS ===");
        OpModeData.telemetry.addData("Left Motor", "%.1f", leftMotor.getCurrentPosition());
        OpModeData.telemetry.addData("Right Motor", "%.1f", rightMotor.getCurrentPosition());
        OpModeData.telemetry.addData("Left Power", "%.3f", leftMotor.getPower());
        OpModeData.telemetry.addData("Right Power", "%.3f", rightMotor.getPower());

        OpModeData.telemetry.update();

         */
    }
}
