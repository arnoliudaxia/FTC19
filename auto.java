  /* Copyright (c) 2017 FIRST. All rights reserved.
   *
   * Redistribution and use in source and binary forms, with or without modification,
   * are permitted (subject to the limitations in the disclaimer below) provided that
   * the following conditions are met:
   *
   * Redistributions of source code must retain the above copyright notice, this list
   * of conditions and the following disclaimer.
   *
   * Redistributions in binary form must reproduce the above copyright notice, this
   * list of conditions and the following disclaimer in the documentation and/or
   * other materials provided with the distribution.
   *
   * Neither the name of FIRST nor the names of its contributors may be used to endorse or
   * promote products derived from this software without specific prior written permission.
   *
   * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
   * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
   * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
   * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
   * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
   */

  package org.firstinspires.ftc.teamcode;

  import com.qualcomm.hardware.ams.AMSColorSensor;
  import com.qualcomm.hardware.bosch.BNO055IMU;
  import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
  import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
  import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
  import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
  import com.qualcomm.robotcore.hardware.DcMotor;
  import com.qualcomm.robotcore.hardware.Servo;
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
  import org.firstinspires.ftc.robotcore.external.navigation.Position;
  import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
  import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
  import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
  import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

  import java.util.List;
  import java.util.Locale;

  @Autonomous(name="yindi", group="Linear Opmode")
  public class auto extends LinearOpMode {

      //region 定义马达
      private ElapsedTime runtime = new ElapsedTime();
      private DcMotor motor_zuoqian;
      private DcMotor motor_youqian;
      private DcMotor motor_zuohou;
      private DcMotor motor_youhou;
      private DcMotor motor_xuangua;
      private DcMotor motor_tian;
      private double yp,rp;
      private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
      private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
      private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
      double time,time1;

      BNO055IMU imu;
      Orientation angles;
      Acceleration gravity;


      private static final String VUFORIA_KEY = "AVdOcwf/////AAABmbswlTQtOUzMptGVnSjAeKVGwl2iTTGiYmbkrb9oeSJtSS+XVedw0nNaOw/hrg4SOQBWNJGQMsZF+haAaLAVN+O9fn4WeUxvmd4DfuQETZB8za8bCnZR1nrEw2ywKVhNPOIbcSeUtSQ1Uf58yq6Fb3+QZBQKnmU0CDSbdNjhMqCCiBy1Er+aIVR5P8SmSIOS0zHgZITgdbwxhoNFiCP0sy7AvRKXYoR/IbJicqEnzibPT7zgKDWD2fq2B46knjD27JMnN+t+O/1tE9sujbZsWucqt7AcTy2bv1qLV14iK0hUpyOOY9hd3MDCiL93on/MHKYbWd2qX+FgVYtTNtH77Jg/uXsIrpjqIbqSdsFvTf7i";

      private VuforiaLocalizer vuforia;
      private TFObjectDetector tfod;

      float gold,silver1=0,silver2=0;
      boolean goldDe=false,isAim=false;
      boolean isleft;
      double angle=0;
      //endregion

      @Override
      public void runOpMode() {

          telemetry.addData("Status", "初始化完毕");
          initCamJet();


          motor_zuoqian = hardwareMap.get(DcMotor.class, "motor_zuoqian");
          motor_youqian = hardwareMap.get(DcMotor.class, "motor_youqian");
          motor_zuohou = hardwareMap.get(DcMotor.class, "motor_zuohou");
          motor_youhou = hardwareMap.get(DcMotor.class, "motor_youhou");
          motor_xuangua = hardwareMap.get(DcMotor.class, "motor_xuangua");

          motor_zuoqian.setDirection(DcMotor.Direction.FORWARD);
          motor_zuohou.setDirection(DcMotor.Direction.FORWARD);
          motor_youqian.setDirection(DcMotor.Direction.REVERSE);
          motor_youhou.setDirection(DcMotor.Direction.REVERSE);


          initIMU();

          telemetry.update();
          // Wait for the game to start (driver presses PLAY)
          waitForStart();
          runtime.reset();

          // run until the end of the match (driver presses STOP)

//region 着陆
          // region 空降
          while(opModeIsActive()){
              gravity =imu.getGravity();
              double yA=gravity.yAccel;
              if (yA < -0.2) {
                  motor_xuangua.setPower(-0.7);
                  telemetry.addData("瞬时速度",-0.7);
                  composeTelemetry();
                  telemetry.update();
              }
              else {
                  motor_xuangua.setPower(0);
                  time=time1 = runtime.milliseconds();
                  telemetry.addData(">","空降已完成");
                  telemetry.update();
                  break;
                  }
          }
            //endregion
          //region 脱离
          while(opModeIsActive()) {
              if (runtime.milliseconds() - time < 500)
                  runMotor(formal1.MotorMode.Forward);
              else {
                  runMotor(formal1.MotorMode.Stop);
                  time = runtime.milliseconds();
                  telemetry.addData(">","完成脱离");
                  telemetry.update();
                  break;
              }
          }
           //endregion
          //region 回收
          while(opModeIsActive()){
             if (runtime.milliseconds()-time<=time1)
                  motor_xuangua.setPower(0.8);
             else{
                  motor_xuangua.setPower(0);
                  telemetry.addData(">","完成回收");
                  telemetry.update();
                  break;
             }
          }
           //endregion
          telemetry.addData(">>","完美着陆");
          telemetry.update();
//endregion
//region 取样
          telemetry.addData(">>>","进入取样阶段");
          telemetry.update();
          //region 瞄准
          if (opModeIsActive()) {
              /** Activate Tensor Flow Object Detection. */
              if (tfod != null) {
                  tfod.activate();
                  angle = 100;
                  telemetry.addData(">","超级瞄准已部署");
              }
          }
          while (opModeIsActive()){

              List<Recognition> updatedRecognition =tfod.getUpdatedRecognitions();
              if(updatedRecognition != null) {
                  for(Recognition recognition:updatedRecognition) {
                          if(recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                              gold = recognition.getLeft();
                              goldDe = true;
                              angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                              if(angle < 0)isleft = true;
                              else isleft=false;
                              telemetry.addData("<","发现敌人");
                              telemetry.addData("角度:",angle);

                          }
                  }
                  if(goldDe) {
                      if(angle == 100)continue;
                      if(angle <= 4 && angle >= -4)isAim = true;
                      if (!isAim) {
                          if (isleft)
                          runMotor(formal1.MotorMode.AntiClock);
                          else
                              runMotor(formal1.MotorMode.Clock);
                          telemetry.addData(">","瞄准中........");
                      }
                      if(isAim) {
                          runMotor(formal1.MotorMode.Stop);
                          time = runtime.milliseconds();
                          telemetry.addData(">","瞄准完成");
                          telemetry.update();
                          break;
                      }
                  }
              }
              telemetry.update();
          }
          //endregion
          //region 攻击
          while (opModeIsActive()){
              if (time - runtime.milliseconds() <= 6000){
                  runMotor(formal1.MotorMode.Right);
              }else{
                  runMotor(formal1.MotorMode.Stop);
                  time = runtime.milliseconds();
                  break;
              }
          }

          //endregion

//endregion
//region 宣示主权+停止
          while(opModeIsActive()){
              if(runtime.milliseconds() - time == 1){
                  motor_tian.setPower(-1);
              }else{
                  motor_tian.setPower(0);
                  telemetry.addData("我方主权", "已宣示");
                  telemetry.update();
                  break;
              }
          }


//endregion
//region 停止
          if(opModeIsActive()){
          runMotor(0,0,0,0);
          motor_tian.setPower(0);
          telemetry.addData("自动阶段","完成");
      }

//endregion
//              // Show the elapsed game time and wheel power.
              telemetry.addData("Status", "Run Time: " + runtime.toString());
              telemetry.addData("Motors", "zuoqian (%.2f), youqian (%.2f),zuohou (%.2f),youhou (%.2f),xuangua（%.2f）", motor_zuoqian.getPower(), motor_youqian.getPower(), motor_zuohou.getPower(), motor_youhou.getPower(),motor_xuangua.getPower());
              telemetry.addData("TIME:", runtime.seconds());
              telemetry.update();
          }

      public void runMotor(double LF, double LB, double RF, double RB) {
          motor_zuoqian.setPower(LF);
          motor_zuohou.setPower(LB);
          motor_youhou.setPower(RB);
          motor_youqian.setPower(RF);
      }
      public void runMotor(formal1.MotorMode mode) {
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
                  motor_zuohou.setPower(0);
                  motor_youqian.setPower(0);
                  motor_youhou.setPower(0);
                  motor_tian.setPower(0);
              case Clock:
                  motor_zuoqian.setPower(0.23);
                  motor_zuohou.setPower(0.23);
                  motor_youqian.setPower(-0.23);
                  motor_youhou.setPower(-0.23);
              case AntiClock:
                  motor_zuoqian.setPower(-0.23);
                  motor_zuohou.setPower(-0.23);
                  motor_youqian.setPower(0.23);
                  motor_youhou.setPower(0.23);
          }

      }
      private  boolean panduan(double input,double standard,double wucha) {
          if(Math.abs(input-standard)<wucha)return true;
          else return false;
      }
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
      public void initIMU(){
          BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
          parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
          parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
          parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
          parameters.loggingEnabled      = true;
          parameters.loggingTag          = "IMU";
          parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
          imu = hardwareMap.get(BNO055IMU.class, "imu");
          imu.initialize(parameters);
          imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
      }
      private void initCamJet() {
          //region initVuforia
          VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

          parameters.vuforiaLicenseKey = VUFORIA_KEY;
          parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

          //  Instantiate the Vuforia engine
          vuforia = ClassFactory.getInstance().createVuforia(parameters);
          //endregion
          //region initTfod
          int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                  "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
          TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
          tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
          tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

          //endregion
      }
  }

