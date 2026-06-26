package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
public class autotest extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         FrontRight  = null; 
    private DcMotor         BackRight  = null;     
    private DcMotor         BackLeft   = null;
    private DcMotor         FrontLeft  = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.1 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.1;
    static final double     TURN_SPEED              = 0.1;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables

        FrontRight  = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackLeft  = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // telemetry.addData("Starting at",  "%7d :%7d",
        //                   leftDrive.getCurrentPosition(),
        //                   rightDrive.getCurrentPosition());
        // telemetry.update();

        waitForStart();

      
        encoderDrive(DRIVE_SPEED,  48,  48, 48, 48, 500);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   48, -48, -48, 48, 400);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, -24, -24, 400);  // S3: Reverse 24 Inches with 4 Sec timeout
        

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double FrontleftInches, double FrontrightInches, double BackrightInches,double BackleftInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = FrontLeft.getCurrentPosition() + (int)(FrontleftInches * COUNTS_PER_INCH);
            newFrontRightTarget = FrontRight.getCurrentPosition() + (int)(FrontrightInches * COUNTS_PER_INCH);
            newBackLeftTarget = BackLeft.getCurrentPosition() + (int)(BackleftInches * COUNTS_PER_INCH);
            newBackRightTarget = BackRight.getCurrentPosition() + (int)(BackrightInches * COUNTS_PER_INCH);
            
            FrontLeft.setTargetPosition(newFrontLeftTarget);
            FrontRight.setTargetPosition(newFrontRightTarget);
            BackLeft.setTargetPosition(newBackLeftTarget);
            BackRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            FrontLeft.setPower(Math.abs(speed));
            FrontRight.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));
            BackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy())) {
                    telemetry.addData("E","E");
                    telemetry.update();
                // Display it for the driver.
            //     telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
            //     telemetry.addData("Currently at",  " at %7d :%7d",
            //                                 leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
            //     telemetry.update();
            }
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("One movement","finished");
            telemetry.update();
            sleep(250); 
        }
