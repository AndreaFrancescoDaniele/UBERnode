package com.example.ubernode;

        import android.app.Activity;
        import android.content.SharedPreferences;
        import android.graphics.Color;
        import android.hardware.Sensor;
        import android.hardware.SensorEvent;
        import android.hardware.SensorEventListener;
        import android.hardware.SensorManager;
        import android.os.Bundle;
        import android.preference.PreferenceManager;
        import android.view.View;
        import android.widget.ProgressBar;
        import android.widget.ScrollView;
        import android.widget.TextView;
        import android.widget.ToggleButton;
        import android.widget.ViewSwitcher;

        import java.math.BigDecimal;
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
    private final int calibrationDuration = 3000; //3000 mS = 3 sec
    private int calibrationProgress = 1; //at the end, the value will be: 3001 mS

    //Other objects
    private SensorManager sensorManager;
    private DataSender dataSender;
    private SharedPreferences preferences;
    private SharedPreferences.Editor preferencesEditor;
    private LinkedList<String> consoleBuffer;
    private String sensorsStatusString;
    private String connectionStatusString;

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
    private final float sett_upperValue = 10.0f;
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
        sensorsStatusEnum = SensorsStatus.OFFLINE;
        connectionStatusEnum = ConnectionStatus.NOT_CONNECTED;
        sensorsStatusString = getString(R.string.sensors_offline);
        connectionStatusString = getString(R.string.net_notconnected);
        //Concurrency
        consoleSemaphore = new Semaphore(1);
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
        printToConsole(getString(R.string.mode_calibrating)+"\n");
        sensorsStatusString = getString(R.string.sensors_calibrating);
        sensorsStatusEnum = SensorsStatus.CALIBRATING;
        //Create the Preferences file
        preferences = PreferenceManager.getDefaultSharedPreferences(this);
        preferencesEditor = preferences.edit();
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
    public void onAccuracyChanged(Sensor sensor, int accuracy) { /*Do nothing*/  }

    private void switchToReadyState(){
        sensorsStatusEnum = SensorsStatus.SENDING_DATA;
        if(dataSender == null){
            dataSender = new DataSender( this, readStringFromSettings("rosNodeIP"), (int)readFloatFromSettings("rosNodePort") );
            dataSenderAndAppender.clearPerformersList();
            dataSenderAndAppender.addPerformer(dataSender);
        }
        dataSender.connect();
    }//switchToReadyState

    public void onSettingsClick(View view){
        viewSwitcher.showNext();
        if( !readStringFromSettings("upsetLimit").equals("null") ){
            refillSettingsFields();
        }
    }//onSettingsClick

    public void onSettingsConnectClick(View view){
        //Connect dataSender
        writeToSettings( "rosNodeIP", rosNodeIP.getText().toString() );
        writeToSettings( "rosNodePort", rosNodePort.getText().toString() );
        if( connectionStatusEnum == ConnectionStatus.NOT_CONNECTED ){
            if(dataSender != null){
                dataSender.close();
            }
            dataSender = null;
        }
        onSettingsCloseClick(null); //close Settings
    }//onSettingsConnectClick

    public void onSettingsCloseClick(View view){
        viewSwitcher.showPrevious();
    }//onSettingsCloseClick

    public void onSettingsSaveClick(View view){
        //Update settings
        writeToSettings( "rosNodeIP", rosNodeIP.getText().toString() );
        writeToSettings( "rosNodePort", rosNodePort.getText().toString() );
        writeToSettings( "upsetLimit", upsetLimit.getText().toString() );
        writeToSettings( "radius", radius.getText().toString() );
        writeToSettings( "limit", limit.getText().toString() );
        writeToSettings( "upperValue", upperValue.getText().toString() );
        writeToSettings( "step", step.getText().toString() );
    }//onSettingsSaveClick

    public void onRestoreDefaultButtonClick(View view){
        //Restore default setting values
        writeToSettings( "rosNodeIP", sett_rosNodeIP );
        writeToSettings( "rosNodePort", sett_rosNodePort+"" );
        writeToSettings( "upsetLimit", sett_upsetLimit+"" );
        writeToSettings( "radius", sett_radius+"" );
        writeToSettings( "limit", sett_limit+"" );
        writeToSettings( "upperValue", sett_upperValue+"" );
        writeToSettings( "step", sett_step+"" );
        refillSettingsFields();
    }//onRestoreDefaultButtonClick

    private void refillSettingsFields(){
        //Refill settings
        rosNodeIP.setText( readStringFromSettings("rosNodeIP") );
        rosNodePort.setText( readStringFromSettings("rosNodePort") );
        upsetLimit.setText( readStringFromSettings("upsetLimit") );
        radius.setText( readStringFromSettings("radius") );
        limit.setText( readStringFromSettings("limit") );
        upperValue.setText( readStringFromSettings("upperValue") );
        step.setText( readStringFromSettings("step") );
    }//refillSettingsFields

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



    //Settings Methods
    private String readStringFromSettings(String key){
        return preferences.getString(key,"null");
    }//readStringFromSettings

    private float readFloatFromSettings(String key){
        //Read float value
        try{
            return Float.parseFloat( readStringFromSettings(key) );
        }catch(NumberFormatException e){
            return -1f;
        }
    }//readStringFromSettings

    private void writeToSettings(String key, String value){
        preferencesEditor.putString(key,value);
        preferencesEditor.commit();
    }//writeToSettings


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
            consoleBuffer.add(message);
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
                    switchToReadyState();
                    //change clock speed
                    calibrationController.setClockDuration(clock70ms);
                }
            }
        }else if(sensorsStatusEnum == SensorsStatus.SENDING_DATA){
            if(dataSender == null){
                //Connect
                switchToReadyState(); //to reconnect
            }
            //connection lost?
            if( !dataSender.isConnected() ){
                connectionStatusEnum = ConnectionStatus.NOT_CONNECTED;
            }else{
                connectionStatusEnum = ConnectionStatus.CONNECTED;
            }
            //Send new data to dataSender
            dataSender.appendData( buildMessage() );
        }
        //With any clock speed
        canUpdateCoordinates = true;
    }//perform

    private String buildMessage(){
        //Message structure: $<x-coor>|<y-coor>$
        return "$" + coordinates[0] + "|" + coordinates[1] + "$";
    }//buildMessage


}//MainActivity
