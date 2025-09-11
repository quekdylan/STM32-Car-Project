package com.example.mdpapp.activities;

import android.content.ComponentName;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.IBinder;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;
import android.util.Log;

import androidx.appcompat.app.AppCompatActivity;

import com.example.mdpapp.R;
import com.example.mdpapp.services.BluetoothService;

import org.json.JSONObject;

public class ChatActivity extends AppCompatActivity {

    private TextView tvMessages, tvRobotStatus, tvStatusLabel;
    private EditText etMessage;
    private Button btnSend, btnClear;
    private Button btnForward, btnReverse, btnLeft, btnRight;

    private BluetoothService bluetoothService;

    private final BluetoothService.OnMessageReceivedListener messageListener = message ->
            runOnUiThread(() -> tvMessages.append(message));

    private final BluetoothService.OnMessageReceivedListener statusListener = message ->
    {
        runOnUiThread(() -> {
            // Filter only status messages
            if (message.startsWith("{\"status\":")) {
                String status = extractStatus(message);
                if (status != null) {
                    // Overwrite TextView content
                    tvRobotStatus.setText(status);
                }
            }
        });
    };

    private String extractStatus(String message) {
        try {
            JSONObject obj = new JSONObject(message);
            Log.d("Extract Status", "Extracted status: " + obj.getString("status"));
            return obj.getString("status");
        } catch (Exception e) {
            Log.d("ChatActivity", "Failed to parse status JSON: " + e.getMessage());
            return null;
        }
    }

    private boolean isServiceBound = false;

    private final ServiceConnection serviceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            BluetoothService.LocalBinder binder = (BluetoothService.LocalBinder) service;
            bluetoothService = binder.getService();
            bluetoothService.addListener(messageListener);
            bluetoothService.addListener(statusListener);
            bluetoothService.startAcceptThread();
            isServiceBound = true;
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            bluetoothService = null;
            isServiceBound = false;
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_chat);

        // Initialize UI
        tvMessages = findViewById(R.id.tvMessages);
        etMessage = findViewById(R.id.etMessage);
        btnSend = findViewById(R.id.btnSend);
        btnClear = findViewById(R.id.btnClear);
        tvRobotStatus = findViewById(R.id.tvRobotStatus);
        tvStatusLabel = findViewById(R.id.tvStatusLabel);

        // Directional buttons
        btnForward = findViewById(R.id.btnForward);
        btnReverse = findViewById(R.id.btnReverse);
        btnLeft = findViewById(R.id.btnLeft);
        btnRight = findViewById(R.id.btnRight);



        // Bind to BluetoothService
        Intent serviceIntent = new Intent(this, BluetoothService.class);
        bindService(serviceIntent, serviceConnection, BIND_AUTO_CREATE);

        // Send button
        btnSend.setOnClickListener(v -> {
            if (!isServiceBound) {
                Toast.makeText(this, "Bluetooth service not ready", Toast.LENGTH_SHORT).show();
                return;
            }

            String message = etMessage.getText().toString().trim();
            if (!message.isEmpty() && bluetoothService.getConnectedThread() != null) {
                bluetoothService.getConnectedThread().write(message.getBytes());
                etMessage.setText("");
            } else {
                Toast.makeText(this, "No device connected", Toast.LENGTH_SHORT).show();
            }
        });

        // Clear button
        btnClear.setOnClickListener(v -> etMessage.setText(""));

        // Directional button listeners
        btnForward.setOnClickListener(v -> sendMovementCommand("f"));
        btnReverse.setOnClickListener(v -> sendMovementCommand("r"));
        btnLeft.setOnClickListener(v -> sendMovementCommand("tl"));
        btnRight.setOnClickListener(v -> sendMovementCommand("tr"));
    }

    @Override
    protected void onStart() {
        super.onStart();
        if (bluetoothService != null) {
            bluetoothService.addListener(messageListener);
        }
    }

    @Override
    protected void onStop() {
        super.onStop();
        if (bluetoothService != null) {
            bluetoothService.removeListener(messageListener);
            bluetoothService.removeListener(statusListener);
        }
    }
    private void sendMovementCommand(String command) {
        if (!isServiceBound) {
            Toast.makeText(this, "Bluetooth service not ready", Toast.LENGTH_SHORT).show();
            return;
        }

        if (bluetoothService.getConnectedThread() != null) {
            bluetoothService.getConnectedThread().write(command.getBytes());
        } else {
            Toast.makeText(this, "No device connected", Toast.LENGTH_SHORT).show();
        }
    }


}
