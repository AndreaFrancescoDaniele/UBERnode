package net.cloud4service.ubernode;

        import android.app.Activity;
        import android.graphics.Color;
        import android.hardware.Sensor;
        import android.hardware.SensorEvent;
        import android.hardware.SensorEventListener;
        import android.hardware.SensorManager;
        import android.os.Bundle;
        import android.view.View;
        import android.view.WindowManager;
        import android.widget.ProgressBar;
        import android.widget.ScrollView;
        import android.widget.TextView;
        import android.widget.ToggleButton;
        import android.widget.ViewSwitcher;

        import java.math.BigDecimal;
        import java.text.SimpleDateFormat;
        import java.util.Date;
        import java.util.LinkedList;
        import java.util.concurrent.Semaphore;

public class MainActivity extends Activity implements SensorEventListener, PerformedByClock, Console {

    //Sensors data and configuration
    private float[] coordinates = new float[3];
    private float[] rawCoordinates = new float[3];
    private boolean rollEnabled = true;
    private boolean pitchEnabled = true;
    private boolean canUpdateCoordinates = false;
    private SensorsStatus sensorsStatusEnum;
    private ConnectionStatus connectionStatusEnum;
    private ActiveView activeViewEnum;
    private final int calibrationDuration = 3000; //3000 mS = 3 sec
    private int calibrationProgress = 1; //at the end, the value will be: 3001 mS
    private boolean connectionAllowed = false;

    //Other objects
    private SensorManager sensorManager;
    private DataSender dataSender;
    private LinkedList<String> consoleBuffer;
    private String sensorsStatusString;
    private String connectionStatusString;
    private SimpleDateFormat simpleDateFormat;
    private AppPreferences appPreferences;

    //Graphic object
    private TextView xCoor;
    private TextView yCoor;
    private TextView zCoor;
    private ProgressBar xCoorProgressBar;
    private ProgressBar yCoorProgressBar;
    private ProgressBar zCoorProgressBar;
    private ViewSwitcher viewSwitcher;
    private ScrollView scroller;
    private TextView console;
    private TextView sensorsStatus;
    private TextView connectionStatus;
    //Settings view
    private TextView rosNodeIP;
    private TextView rosNodePort;
    private TextView upsetLimit;
    private TextView radius;
    private TextView limit;
    private TextView upperValue;
    private TextView step;

    //Default settings
    private final String sett_rosNodeIP = "192.168.43.124";
    private final int sett_rosNodePort = 8080;
    private final float sett_upsetLimit = 4.0f;
    private final float sett_radius = 2.0f;
    private final float sett_limit = 6.5f;
    private final float sett_upperValue = 10f;
    private final float sett_step = 0.5f;

    //ClocksTimes
    private final int clock100ms = 100; //100mS = 1/10 sec | CalibrationController
    private final int clock70ms = 70; //70mS = 7/100 sec | DataSender & DataAppend
    //Clocks
    private Clock calibrationController;
    private Clock dataSenderAndAppender;

    //Concurrency
    private Semaphore consoleSemaphore;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        //==================================
        //=>Default Code
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        //<=Default Code
        //==================================
        //=>My Code
        //Get graphic objects
        xCoor = (TextView)findViewById(R.id.xCoor);
        yCoor = (TextView)findViewById(R.id.yCoor);
        zCoor = (TextView)findViewById(R.id.zCoor);
        xCoorProgressBar = (ProgressBar)findViewById(R.id.xCoorProgressBar);
        yCoorProgressBar = (ProgressBar)findViewById(R.id.yCoorProgressBar);
        zCoorProgressBar = (ProgressBar)findViewById(R.id.zCoorProgressBar);
        viewSwitcher = (ViewSwitcher)findViewById(R.id.viewSwitcher);
        console = (TextView)findViewById(R.id.console);
        scroller = (ScrollView)findViewById(R.id.scroller);
        sensorsStatus = (TextView)findViewById(R.id.sensorsStatus);
        connectionStatus = (TextView)findViewById(R.id.connectionStatus);
        //Settings view graphic objects
        rosNodeIP = (TextView)findViewById(R.id.rosNodeIP);
        rosNodePort = (TextView)findViewById(R.id.rosNodePort);
        upsetLimit = (TextView)findViewById(R.id.upsetLimit);
        radius = (TextView)findViewById(R.id.radius);
        limit = (TextView)findViewById(R.id.limit);
        upperValue = (TextView)findViewById(R.id.upperValue);
        step = (TextView)findViewById(R.id.step);
        //Enum e States
        activeViewEnum = ActiveView.MAIN;
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        sensorsStatusEnum = SensorsStatus.OFFLINE;
        connectionStatusEnum = ConnectionStatus.NOT_CONNECTED;
        sensorsStatusString = getString(R.string.sensors_offline);
        connectionStatusString = getString(R.string.net_notconnected);
        //Create preferences
        appPreferences = new AppPreferences(this);
        //send preferences to DataSmoother
        DataSmoother.setAppPreferences( appPreferences );
        if( appPreferences.contains("preferencesWrittenFlag") ){
            //settings redefined by user
            DataSmoother.reloadParameters();
            preferencesUpdated( appPreferences.readString("upperValue") );
        }
        //Concurrency
        consoleSemaphore = new Semaphore(1);
        //Others objects
        simpleDateFormat = new SimpleDateFormat("HH:mm:ss");
        //create consoleBuffer
        consoleBuffer = new LinkedList<String>();
        //Create dataSender
        dataSender = new DataSender( this, sett_rosNodeIP, sett_rosNodePort );
        //create Clocks
        calibrationController = new Clock(clock100ms);
        dataSenderAndAppender = new Clock(clock70ms);
        //register performer to a clock
        calibrationController.addPerformer(this);
        dataSenderAndAppender.addPerformer(dataSender);
        //start Clocks
        calibrationController.start();
        dataSenderAndAppender.start();
        //configure app
        printToConsole("#====================== \n Console: \n");
        printToConsole(getString(R.string.app_name) + " - v" + getString(R.string.app_version));
        printToConsole("Developed by: " + getString(R.string.app_developer));
        printToConsole(getString(R.string.app_launched) + '\n');
        //create sensorManager
        sensorManager=(SensorManager)getSystemService(SENSOR_SERVICE);
        //register listener
        sensorManager.registerListener(this,
                sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_NORMAL);
        //Update status and labels
        printToConsole(getString(R.string.calibration_mode));
        printToConsole(getString(R.string.calibration_mode_warning)+"\n");
        sensorsStatusString = getString(R.string.sensors_calibrating);
        sensorsStatusEnum = SensorsStatus.CALIBRATING;
        //<=My Code
        //==================================
    }//onCreate


    @Override
    public void onPause(){
        //Disable axis, send (0, 0) all the time
        rollEnabled = false;
        pitchEnabled = false;
        super.onPause();
    }//onPause

    @Override
    public void onResume(){
        super.onResume();
        //Enable axis
        rollEnabled = true;
        pitchEnabled = true;
    }//onPause

    @Override
    public void onStop(){
        //Close connection
        dataSender.close();
        super.onStop();
    }//onPause

    @Override
    public void onBackPressed(){
        if(activeViewEnum == ActiveView.SETTINGS){
            onSettingsCloseClick(null);
        }else{
            finish();
        }
    }//onBackPressed




    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) { /*Do nothing*/  }

    private void switchToReadyState(){
        sensorsStatusEnum = SensorsStatus.SENDING_DATA;
        if(dataSender == null){
            dataSender = new DataSender( this, appPreferences.readString("rosNodeIP"), (int)appPreferences.readFloat("rosNodePort") );
            dataSenderAndAppender.clearPerformersList();
            dataSenderAndAppender.addPerformer(dataSender);
        }
        if(connectionAllowed){
            dataSender.connect();
        }
    }//switchToReadyState

    public void onSettingsClick(View view){
        //disable axis
        rollEnabled = false;
        pitchEnabled = false;
        //change view
        viewSwitcher.showNext();
        activeViewEnum = ActiveView.SETTINGS;
        if( appPreferences.contains("preferencesWrittenFlag") ){
            refillSettingsFields();
        }
    }//onSettingsClick

    public void onSettingsConnectClick(View view){
        //Connect dataSender
        appPreferences.writeString( "rosNodeIP", rosNodeIP.getText().toString() );
        appPreferences.writeString( "rosNodePort", rosNodePort.getText().toString() );
        appPreferences.commitChanges();
        if( connectionStatusEnum == ConnectionStatus.NOT_CONNECTED ){
            if(dataSender != null){
                dataSender.close();
            }
            dataSender = null;
            connectionAllowed = true;
        }
        onSettingsCloseClick(null); //close Settings
    }//onSettingsConnectClick

    public void onSettingsCloseClick(View view){
        viewSwitcher.showPrevious();
        activeViewEnum = ActiveView.MAIN;
        //enable axis
        rollEnabled = true;
        pitchEnabled = true;
    }//onSettingsCloseClick

    public void onSettingsSaveClick(View view){
        //Update settings
        appPreferences.writeString( "rosNodeIP", rosNodeIP.getText().toString() );
        appPreferences.writeString( "rosNodePort", rosNodePort.getText().toString() );
        appPreferences.writeString( "upsetLimit", upsetLimit.getText().toString() );
        appPreferences.writeString( "radius", radius.getText().toString() );
        appPreferences.writeString( "limit", limit.getText().toString() );
        appPreferences.writeString( "upperValue", upperValue.getText().toString() );
        appPreferences.writeString( "step", step.getText().toString() );
        appPreferences.commitChanges();
        preferencesUpdated( upperValue.getText().toString()/*progBarMax*/ );
        DataSmoother.reloadParameters();
    }//onSettingsSaveClick

    public void onRestoreDefaultButtonClick(View view){
        //Restore default setting values
        appPreferences.writeString( "rosNodeIP", sett_rosNodeIP );
        appPreferences.writeString( "rosNodePort", sett_rosNodePort+"" );
        appPreferences.writeString( "upsetLimit", sett_upsetLimit+"" );
        appPreferences.writeString( "radius", sett_radius+"" );
        appPreferences.writeString( "limit", sett_limit+"" );
        appPreferences.writeString( "upperValue", sett_upperValue+"" );
        appPreferences.writeString( "step", sett_step+"" );
        appPreferences.commitChanges();
        preferencesUpdated( sett_upperValue+""/*progBarMax*/ );
        DataSmoother.reloadParameters();
        refillSettingsFields();
    }//onRestoreDefaultButtonClick

    private void refillSettingsFields(){
        //Refill settings
        rosNodeIP.setText( appPreferences.readString("rosNodeIP") );
        rosNodePort.setText( appPreferences.readString("rosNodePort") );
        upsetLimit.setText( appPreferences.readString("upsetLimit") );
        radius.setText( appPreferences.readString("radius") );
        limit.setText( appPreferences.readString("limit") );
        upperValue.setText( appPreferences.readString("upperValue") );
        step.setText( appPreferences.readString("step") );
    }//refillSettingsFields

    private void preferencesUpdated(String progBarMax){
        //make the progress bars adequate to the upperLimit
        int max = (int)sett_upperValue;
        try{
            max = (int) Float.parseFloat(progBarMax);
        }catch(Exception e){/*do nothing*/}
        xCoorProgressBar.setMax( max );
        yCoorProgressBar.setMax( max );
    }//preferencesUpdated

    public void onRollToggleClicked(View view){
        boolean flag = ((ToggleButton) view).isChecked();
        rollEnabled = flag;
        if(flag){
            //Enabled
            printToConsole(getString(R.string.config_rollEnabled));
        }else{
            //Disabled
            printToConsole(getString(R.string.config_rollDisabled));
            printToConsole(getString(R.string.config_rollDisabled2));
        }
    }//onRollToggleClicked

    public void onPitchToggleClicked(View view){
        boolean flag = ((ToggleButton) view).isChecked();
        pitchEnabled = flag;
        if(flag){
            //Enabled
            printToConsole(getString(R.string.config_pitchEnabled));
        }else{
            //Disabled
            printToConsole(getString(R.string.config_pitchDisabled));
            printToConsole(getString(R.string.config_pitchDisabled2));
        }
    }//onPitchToggleClicked


    @Override
    public void onSensorChanged(SensorEvent event){
        if(canUpdateCoordinates){
            //lock
            canUpdateCoordinates = false;
            //check sensor type
            if(event.sensor.getType()==Sensor.TYPE_ACCELEROMETER){
                //update coordinates
                rawCoordinates[0] = ( (rollEnabled)? event.values[0] : 0 );
                rawCoordinates[1] = ( (pitchEnabled)? event.values[1] : 0 );
                rawCoordinates[2] = event.values[2];
                //Call dataSmoother
                coordinates = DataSmoother.smooth( rawCoordinates[0], rawCoordinates[1], rawCoordinates[2] );
            }
            //Refresh data on screen
            refreshDataOnScreen();
            //flush consoleBuffer
            flushDataToConsole();
        }else{
            return; //doNothing
        }
    }//onSensorChanged

    private void refreshDataOnScreen(){
        //refresh text
        xCoor.setText(""+coordinates[0]);
        yCoor.setText(""+coordinates[1]);
        zCoor.setText(""+coordinates[2]);
        //refresh progressBars
        xCoorProgressBar.setProgress( (int) Math.abs(coordinates[0]) );
        yCoorProgressBar.setProgress( (int) Math.abs(coordinates[1]) );
        zCoorProgressBar.setProgress( (int) Math.abs(coordinates[2]) );
        //Status
        if(sensorsStatusEnum == SensorsStatus.SENDING_DATA){
            sensorsStatusString = getString(R.string.sensors_online);
            sensorsStatus.setTextColor(Color.GREEN);
        }
        if(connectionStatusEnum == ConnectionStatus.CONNECTED){
            connectionStatusString = getString(R.string.net_connected);
            connectionStatus.setTextColor(Color.GREEN);
        }else{
            connectionStatusString = getString(R.string.net_notconnected);
            connectionStatus.setTextColor(Color.BLACK);
        }
        //Labels
        sensorsStatus.setText( sensorsStatusString );
        connectionStatus.setText( connectionStatusString );
    }//refreshDataOnScreen


    //Utility Methods
    private float round(float d, int decimalPlace) {
        BigDecimal bd = new BigDecimal(Float.toString(d));
        bd = bd.setScale(decimalPlace, BigDecimal.ROUND_HALF_UP);
        return bd.floatValue();
    }//round

    private void flushDataToConsole(){
        try {
            consoleSemaphore.acquire();
            //=> Sezione critica
            if (consoleBuffer.size() != 0) {
                //there is something to print
                if (console != null) {
                    for (String s : consoleBuffer) {
                        scrollDown();
                        console.append(s + "\n");
                        scrollDown();
                    }
                    consoleBuffer.clear();
                }
            }
            //<= Sezione critica
            consoleSemaphore.release();
        } catch (InterruptedException e) {
            consoleSemaphore.release();
            //do nothing
        }
    }//flushDataToConsole

    @Override
    public void printToConsole(String message){
        try {
            consoleSemaphore.acquire();
            //=> Sezione critica
            consoleBuffer.add("["+simpleDateFormat.format(new Date())+"] "+message);
            //<= Sezione critica
            consoleSemaphore.release();
        } catch (InterruptedException e) {
            consoleSemaphore.release();
            //do nothing
        }
    }//flushDataToConsole

    private void scrollDown() {
        scroller.scrollTo(0, console.getBottom());
    }//scrollDown

    @Override
    public void perform() {
        if (sensorsStatusEnum == SensorsStatus.CALIBRATING) {
            //CalibrationMode
            if (coordinates[0] != 0 || coordinates[1] != 0) {
                //calibration restart
                calibrationProgress = 1;
            } else {
                //calibration in progress
                calibrationProgress += clock100ms;
                sensorsStatusString = getString(R.string.sensors_calibrating)+"  "+round(((float)calibrationProgress)/1000, 2)+" sec";
                if (calibrationProgress > calibrationDuration) {
                    //calibration complete
                    printToConsole(getString(R.string.calibration_mode_complete)+"\n");
                    //change appState
                    switchToReadyState();
                    //change clock speed
                    calibrationController.setClockDuration(clock70ms);
                }
            }
        }else if(sensorsStatusEnum == SensorsStatus.SENDING_DATA){
            if(dataSender == null){
                if(connectionAllowed){
                    //Connect
                    switchToReadyState(); //to reconnect
                }
            }
            //connection lost?
            if( !dataSender.isConnected() ){
                connectionStatusEnum = ConnectionStatus.NOT_CONNECTED;
            }else{
                connectionStatusEnum = ConnectionStatus.CONNECTED;
            }
            if(dataSender != null){
                //Send new data to dataSender
                dataSender.appendData( buildMessage() );
            }
        }
        //With any clock speed
        canUpdateCoordinates = true;
    }//perform

    private String buildMessage(){
        //Message structure: $<x-coor>|<y-coor>$
        return "$" + coordinates[0] + "|" + coordinates[1] + "$";
    }//buildMessage


}//MainActivity
