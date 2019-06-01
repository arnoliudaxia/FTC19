package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

//主要程序
@TeleOp(name="Formal2", group="Iterative Opmode")
public class Formal2 extends OpMode
{
    //region 定义各变量
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor_zuoqian;
             DcMotor motor_zuohou;
             DcMotor motor_youqian;
             DcMotor motor_youhou;
             DcMotor motor_xuanzhuan;
             DcMotor motor_lashen;
             DcMotor motor_xuangua;
             DcMotor motor_tian;
             double power_zuoqian;
             double power_youqian;
             double power_zuohou;
             double power_youhou;
             double power_xuanzhuan;
             double power_lashen;
             double power_xuangua;
             double power_tian;
             double p1lx;
             double p1rx;
             double p1ly;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    enum MotorMode{
        Forward,Back,Left,Right,Stop
    }


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AVdOcwf/////AAABmbswlTQtOUzMptGVnSjAeKVGwl2iTTGiYmbkrb9oeSJtSS+XVedw0nNaOw/hrg4SOQBWNJGQMsZF+haAaLAVN+O9fn4WeUxvmd4DfuQETZB8za8bCnZR1nrEw2ywKVhNPOIbcSeUtSQ1Uf58yq6Fb3+QZBQKnmU0CDSbdNjhMqCCiBy1Er+aIVR5P8SmSIOS0zHgZITgdbwxhoNFiCP0sy7AvRKXYoR/IbJicqEnzibPT7zgKDWD2fq2B46knjD27JMnN+t+O/1tE9sujbZsWucqt7AcTy2bv1qLV14iK0hUpyOOY9hd3MDCiL93on/MHKYbWd2qX+FgVYtTNtH77Jg/uXsIrpjqIbqSdsFvTf7i";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;




    /*
     * Code to run ONCE when the driver hits INIT
     */
    //endregion
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        //region 初始化马达
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor_zuoqian  = hardwareMap.get(DcMotor.class, "motor_zuoqian");
        motor_youqian = hardwareMap.get(DcMotor.class, "motor_youqian");
        motor_zuohou = hardwareMap.get(DcMotor.class, "motor_zuohou");
        motor_youhou = hardwareMap.get(DcMotor.class, "motor_youhou");
        motor_xuanzhuan = hardwareMap.get(DcMotor.class,"motor_xuanzhuan");
        motor_xuangua = hardwareMap.get(DcMotor.class,"motor_xuangua");
        motor_lashen = hardwareMap.get(DcMotor.class,"motor_lashen");
        motor_tian = hardwareMap.get(DcMotor.class,"motor_tian");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motor_zuoqian.setDirection(DcMotor.Direction.FORWARD);
        motor_zuohou.setDirection(DcMotor.Direction.FORWARD);
        motor_youqian.setDirection(DcMotor.Direction.REVERSE);
        motor_youhou.setDirection(DcMotor.Direction.REVERSE);
        //有些马达需要零功率刹车来完成制动的操作
        motor_lashen.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_xuanzhuan.getController().setMotorZeroPowerBehavior(0,DcMotor.ZeroPowerBehavior.BRAKE);
        motor_xuangua.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //endregion
       //region 初始化惯性传感器
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();
        //endregion
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {


    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //region 底盘手动程序
        p1lx = gamepad1.left_stick_x;
        p1ly = gamepad1.left_stick_y;
        p1rx = Range.clip(gamepad1.right_stick_x,-1,1);
        power_zuoqian = Range.clip(p1ly + p1lx + p1rx, -1, 1);
        power_youqian = Range.clip(p1ly - p1lx - p1rx, -1, 1);
        power_zuohou = Range.clip(p1ly - p1lx + p1rx, -1, 1);
        power_youhou = Range.clip(p1ly + p1lx - p1rx, -1, 1);


        if (gamepad2.left_bumper){
            power_xuangua = 0.8;
        }else if(gamepad2.right_bumper){
            power_xuangua = -0.8;
        }else{
            power_xuangua = 0;
        }
        if(gamepad2.x) {
            power_tian = -1;
        }else if (gamepad2.y){
            power_tian = 0;
        }
        power_xuanzhuan = Range.clip(-gamepad2.right_stick_y,-0.8,0.8);
        power_lashen = Range.clip(-gamepad2.left_stick_y,-0.8,0.8);
        motor_zuoqian.setPower(power_zuoqian);
        motor_youqian.setPower(power_youqian);
        motor_zuohou.setPower(power_zuohou);
        motor_youhou.setPower(power_youhou);
        motor_xuanzhuan.getController().setMotorPower(0,power_xuanzhuan);
        motor_xuangua.setPower(power_xuangua);
        motor_lashen.setPower(power_lashen);
        motor_tian.setPower(power_tian);
        //endregion
        // region 输出信息
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "zuoqian (%.2f), youqian (%.2f), zuohou(%.2f), youhou(%.2f),xuanzhuan(%.2f),lashen(%2.2f),xuangua(%.2f),tian(%.2f)", power_zuoqian, power_youqian,power_zuohou,power_youhou,power_xuanzhuan,power_lashen,power_xuangua,power_tian);
        //endregion
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    //以下是一些不参与过程的简化程序
    //region 初始化摄像头和tensorflow
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia()
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    //endregion
    //region 自动程序底盘移动的简化
    public void runMotor(double LF,double LB,double RF,double RB)
    {
        motor_zuoqian.setPower(LF);
        motor_zuohou.setPower(LB);
        motor_youhou.setPower(RB);
        motor_youqian.setPower(RF);
    }
    public void runMotor(MotorMode mode)
    {
        switch (mode) {
            case Forward:
                motor_zuoqian.setPower(1);
                motor_zuohou.setPower(1);
                motor_youqian.setPower(1);
                motor_youhou.setPower(1);
            case Back:
                motor_zuoqian.setPower(-1);
                motor_zuohou.setPower(-1);
                motor_youqian.setPower(-1);
                motor_youhou.setPower(-1);
            case Left:
                motor_zuoqian.setPower(-1);
                motor_zuohou.setPower(1);
                motor_youqian.setPower(1);
                motor_youhou.setPower(-1);
            case Right:
                motor_zuoqian.setPower(1);
                motor_zuohou.setPower(-1);
                motor_youqian.setPower(-1);
                motor_youhou.setPower(1);
            case Stop:
                motor_zuoqian.setPower(0);
                motor_youqian.setPower(0);
                motor_zuohou.setPower(0);
                motor_youhou.setPower(0);
        }
    }
    //endregion
    //region 惯性传感器遥测
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    //endregion

}


