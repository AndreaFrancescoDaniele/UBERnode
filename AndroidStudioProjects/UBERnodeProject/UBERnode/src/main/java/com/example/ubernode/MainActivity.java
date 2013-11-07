package com.example.ubernode;

        import android.app.Activity;
        import android.app.ActionBar;
        import android.app.Fragment;
        import android.hardware.Sensor;
        import android.hardware.SensorEvent;
        import android.hardware.SensorEventListener;
        import android.hardware.SensorManager;
        import android.os.Bundle;
        import android.view.LayoutInflater;
        import android.view.Menu;
        import android.view.MenuItem;
        import android.view.View;
        import android.view.ViewGroup;
        import android.os.Build;
        import android.widget.Button;
        import android.widget.EditText;
        import android.widget.ProgressBar;
        import android.widget.ScrollView;
        import android.widget.TextView;
        import android.widget.ToggleButton;

        import java.math.BigDecimal;
        import java.util.Arrays;

public class MainActivity extends Activity implements SensorEventListener {

    //Sensors data and configuration
    private float[] coordinates = new float[3];
    private boolean rollEnabled = true;
    private boolean pitchEnabled = true;
    private boolean wip = false;

    //Other objects
    private SensorManager sensorManager;

    //Graphic object
    private TextView xCoor;
    private TextView yCoor;
    private TextView zCoor;
    private ProgressBar xCoorProgressBar;
    private ProgressBar yCoorProgressBar;
    private ProgressBar zCoorProgressBar;
    //private EditText console;
    private ScrollView scroller;
    private TextView console;


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
        //console = (EditText)findViewById(R.id.consoleTT);
        console = (TextView)findViewById(R.id.console);
        scroller = (ScrollView)findViewById(R.id.scroller);
        //configure app
        printToConsole( "#====================== \n Console: \n" );
        printToConsole( getString(R.string.app_name)+" - v"+getString(R.string.app_version) );
        printToConsole( "Developed by: "+getString(R.string.app_developer) );
        printToConsole( getString(R.string.app_launched)+'\n' );
        //start clock
        new Clock(this).start();
        //create sensorManager
        sensorManager=(SensorManager)getSystemService(SENSOR_SERVICE);
        //register listener
        sensorManager.registerListener(this,
                sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_NORMAL);
        printToConsole( getString(R.string.mode_calibrating) );
                //<=My Code
        //==================================
    }//onCreate

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        //Do nothing
    }

    public void onSettingsClick(View view){
        printToConsole("Settings!");
    }

    public void onRollToggleClicked(View view){
        boolean flag = ((ToggleButton) view).isChecked();
        rollEnabled = flag;
        if(flag){
            //Enabled
            printToConsole( getString(R.string.config_rollEnabled) );
        }else{
            //Disabled
            printToConsole( getString(R.string.config_rollDisabled) );
            printToConsole( getString(R.string.config_rollDisabled2) );
        }
    }//onRollToggleClicked

    public void onPitchToggleClicked(View view){
        boolean flag = ((ToggleButton) view).isChecked();
        pitchEnabled = flag;
        if(flag){
            //Enabled
            printToConsole( getString(R.string.config_pitchEnabled) );
        }else{
            //Disabled
            printToConsole( getString(R.string.config_pitchDisabled) );
            printToConsole( getString(R.string.config_pitchDisabled2) );
        }
    }//onPitchToggleClicked


    @Override
    public void onSensorChanged(SensorEvent event){
        //lock
        if(wip){
            return; //doNothing
        }else{
            wip = true;
            //check sensor type
            if(event.sensor.getType()==Sensor.TYPE_ACCELEROMETER){
                //update coordinates
                coordinates[0] = ( (rollEnabled)? event.values[0] : 0 );
                coordinates[1] = ( (pitchEnabled)? event.values[1] : 0 );
                coordinates[2] = event.values[2];
                //Call dataSmoother
                coordinates = DataSmoother.smooth( coordinates[0], coordinates[1], coordinates[2] );
            }
            //Refresh data on screen
            refreshDataOnScreen();
            wip = false;
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
    }//refreshDataOnScreen



    //Utility Methods
    private float round(float d, int decimalPlace) {
        BigDecimal bd = new BigDecimal(Float.toString(d));
        bd = bd.setScale(decimalPlace, BigDecimal.ROUND_HALF_UP);
        return bd.floatValue();
    }//round

    private void printToConsole(String str){
        if(console != null){
            scrollDown();
            console.append(str+"\n");
            scrollDown();
        }
    }//printToConsole

    private void scrollDown() {
        scroller.scrollTo(0, console.getBottom()-1);
    }//scrollDown

}//MainActivity
