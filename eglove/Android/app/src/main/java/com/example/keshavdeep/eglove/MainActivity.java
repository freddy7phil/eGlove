package com.example.keshavdeep.eglove;

import android.annotation.TargetApi;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.List;
import java.util.Set;
import java.util.UUID;


public class MainActivity extends AppCompatActivity {

    final Handler handler =new Handler();
    private Runnable messageUpdater;
    private ConnectThread mConnectThread;
    private static String message = "No Message";
    private ConnectedThread mConnectedThread;
    private List<UUID> uuidCandidates;
    private BluetoothSocket mSocket ;
    private BluetoothDevice ourdevice;
    private CharSequence[] cs = {"A","B","C"};

    Boolean read = true;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        Log.d("SOMETHING","first");
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
        BluetoothAdapter mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (mBluetoothAdapter == null) {
            Toast.makeText(MainActivity.this, "Not supported", Toast.LENGTH_SHORT).show();
        } else {
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBtIntent, 1);
        }
        String address = "";
        Set<BluetoothDevice> pairedDevices = mBluetoothAdapter.getBondedDevices();
        if (pairedDevices.size() > 0) {
            for (BluetoothDevice device : pairedDevices) {
                BluetoothDevice mDevice = device;
                if(mDevice.getName().equalsIgnoreCase("BlueZ 5.28")) {
                    TextView myTextView = (TextView) findViewById(R.id.textView);
                    address = mDevice.getAddress();
                    myTextView.setText(message);
                    Log.d("something", message);
                }
            }
        }
        if(!address.isEmpty()) {
            ourdevice = mBluetoothAdapter.getRemoteDevice(address);
        }
        else{
            message = "eGlove is not connected";
            TextView myTextView = (TextView) findViewById(R.id.textView);
            myTextView.setText(message);
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    public void OnCloseConnection(View v)
    {
        read = false;
        try {
            mSocket.close();
            message = "Connection Closed";
        } catch (Exception closeException) {
            message = "Poblem in closing connection";

        }
    }
    public void OnStartConnection(View v)
    {
        read = true;
        final TextView myTextView = (TextView)findViewById(R.id.textView);
        mConnectThread = new ConnectThread(ourdevice);
        mConnectThread.start();
        message = "Connection Initiated";
        messageUpdater = new Runnable() {
            public void run() {
                handler.postDelayed(this, 10);

                if(message.length()>4)
                {
                    myTextView.setTextSize(32);

                }
                else{
                    myTextView.setTextSize(252);

                }
                myTextView.setText(message);

            }
        };
        handler.postDelayed(messageUpdater, 000);

    }
    private class ConnectThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final BluetoothDevice mmDevice;
        BluetoothAdapter mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        public ConnectThread(BluetoothDevice device) {

            BluetoothSocket tmp = null;
            mmDevice = device;
            try {
                final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805f9b34fb");
                tmp = device.createRfcommSocketToServiceRecord(MY_UUID);
                Log.d("something"+tmp,"Keshav");

            } catch (Exception e) {
                Log.d("SOMETHING","UUID Error",e);
            }
            mmSocket = tmp;
            Log.d("SOMETHING","Test");
        }
        @TargetApi(Build.VERSION_CODES.M)
        public void run() {
            //mmSocket.getRemoteDevice().getName()
            //while (read) {
                try {
                    //if (mmSocket.getInputStream().read() > 0) {
                        if (mmSocket.isConnected())
                            Log.d("connected1", "Hello");
                        else {
                            mmSocket.connect();
                            Log.d("connected", "Hello");
                        }
                        mConnectedThread = new ConnectedThread(mmSocket);
                        mConnectedThread.start();
                        message = "Connected to eGlove";
                        //break;

                    //} else {
                    //}
                } catch (IOException connectException) {
                    Log.d("NotConnected", "Hello", connectException);
                    try {
                        mmSocket.close();
                    } catch (IOException closeException) {}
                }
            //}
                    mSocket = mmSocket;

                    //mConnectedThread = new ConnectedThread(mmSocket);
                    //mConnectedThread.start();

                    return;
        }
        public void cancel() {
            try {
                mmSocket.close();
                super.stop();
            } catch (IOException e) {
            }
        }
    }
    private class ConnectedThread extends Thread {

        private final BluetoothSocket mmSocket;
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;

        public ConnectedThread(BluetoothSocket socket) {
            mmSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;
            try {
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException e) {
                Log.d("temp1","Hello2");
            }
            mmInStream = tmpIn;
            mmOutStream = tmpOut;
            Log.d("temp","Hello2");
        }

        public void run() {
            while (read) {
                byte[] buffer = new byte[16284];
                int begin = 0;
                int bytes = 0;
                int check = 0;
                try {
                    bytes = mmInStream.read(buffer);
                } catch (IOException e) {
                    Log.d("Problem", "YOU");
                }
                /*
                if (bytes == 0)
                {
                    message = "Wait until sensors are ready";
                }
                else{
                    message = "Go ahead";
                }
                */
                byte[] buffer2 = new byte[1];
                buffer2[0] = buffer[0];
                String str = new String(buffer2);
                Log.d("Finally", str);
                message = str;
                Log.d("Finally message", message);
            }
            if (!read)
                message = "Connection Closed";

        }
        public void write(byte[] bytes) {
            try {
                mmOutStream.write(bytes);
            } catch (IOException e) {
            }
        }

        public void cancel() {
            //try {
                //mmSocket.close();
                super.stop();
            //} catch (IOException e) {
            //}
        }
    }
}



