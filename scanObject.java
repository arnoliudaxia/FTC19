package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
@TeleOp(name = "testForObject", group = "Concept")
public class scanObject extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String VUFORIA_KEY = "AVdOcwf/////AAABmbswlTQtOUzMptGVnSjAeKVGwl2iTTGiYmbkrb9oeSJtSS+XVedw0nNaOw/hrg4SOQBWNJGQMsZF+haAaLAVN+O9fn4WeUxvmd4DfuQETZB8za8bCnZR1nrEw2ywKVhNPOIbcSeUtSQ1Uf58yq6Fb3+QZBQKnmU0CDSbdNjhMqCCiBy1Er+aIVR5P8SmSIOS0zHgZITgdbwxhoNFiCP0sy7AvRKXYoR/IbJicqEnzibPT7zgKDWD2fq2B46knjD27JMnN+t+O/1tE9sujbZsWucqt7AcTy2bv1qLV14iK0hUpyOOY9hd3MDCiL93on/MHKYbWd2qX+FgVYtTNtH77Jg/uXsIrpjqIbqSdsFvTf7i";

    float gold,silver1=0,silver2=0;
    boolean goldDe=false;
    double angle=0;

    @Override
    public void init() {
        initCamJet();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop()
    {

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        gold=silver1=silver2=0;
        if(tfod!=null)
        {
            tfod.activate();
        }
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if(updatedRecognitions!=null)
        {
            telemetry.addData("# Object Detected",updatedRecognitions.size());
            for(Recognition recognition :updatedRecognitions)
            {
                if(recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                {
                    gold=recognition.getLeft();goldDe=true;
                    angle=recognition.estimateAngleToObject(AngleUnit.DEGREES);
                }
//                if(recognition.getLabel().equals(LABEL_SILVER_MINERAL))
//                {
//                    if(silver1==0)silver1=recognition.getLeft();
//                    else silver2=recognition.getLeft();
//                }
            }
        }
        else{gold=silver1=silver2=-1;}

        telemetry.addData("Gold",gold);
        telemetry.addData("GoldAngle",angle);
        telemetry.addData("Silver1",silver1);
        telemetry.addData("Silver2",silver2);
        telemetry.update();
    }
    private void initCamJet()
    {
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
