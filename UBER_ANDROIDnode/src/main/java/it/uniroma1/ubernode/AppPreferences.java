package it.uniroma1.ubernode;

import android.content.Context;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

/**
 * Created by andrea on 11/10/13.
 */
public class AppPreferences {

    //Objects
    private SharedPreferences preferences;
    private SharedPreferences.Editor preferencesEditor;

    public AppPreferences(Context c){
        //Create the Preferences objects
        preferences = PreferenceManager.getDefaultSharedPreferences( c );
        preferencesEditor = preferences.edit();
    }//AppPreferences


    public boolean contains(String key){
        return preferences.contains(key);
    }//contains

    //Settings Methods
    public String readString(String key, String defaultValue){
        return preferences.getString(key,defaultValue);
    }//readString

    public String readString(String key){
        return readString(key, "null");
    }//readString

    public float readFloat(String key){
        //Read float value
        try{
            return Float.parseFloat( readString(key) );
        }catch(NumberFormatException e){
            return -1f;
        }
    }//readString

    public void writeString(String key, String value){
        preferencesEditor.putString(key,value);
    }//writeToSettings

    public void commitChanges(){
        preferencesEditor.putString("preferencesWrittenFlag", "null");
        preferencesEditor.commit();
    }//commitChanges

}//AppPreferences
