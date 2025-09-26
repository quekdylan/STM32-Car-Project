package com.example.mdpapp.activities;

import android.content.ComponentName;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.IBinder;
import android.preference.PreferenceManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;
import android.util.Log;

import androidx.appcompat.app.AppCompatActivity;

import com.example.mdpapp.R;
import com.example.mdpapp.services.BluetoothService;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

public class ChatActivity extends AppCompatActivity {

    private TextView tvMessages, tvRobotStatus, tvStatusLabel;
    private EditText etMessage;
    private Button btnSend, btnClear, btnPrev, btnNext, btnClearHistory;
    private static final String HISTORY_KEY = "message_history";
    private static final int MAX_HISTORY = 50; // Keep only latest 50 messages
    private final List<String> messageHistory = new ArrayList<>();
    private int historyIndex = -1;  // -1 means "no history selected"

    private BluetoothService bluetoothService;

    private final BluetoothService.OnMessageReceivedListener bluetoothListener = message -> {
        runOnUiThread(() -> {
            tvMessages.append(message + "\n");
            if (message.startsWith("{\"status\":")) {
                String status = extractStatus(message);
                if (status != null) {
                    // Overwrite TextView content
                    tvRobotStatus.setText(status);
                }
            }

            else if (message.startsWith("STATUS:Disconnected from ")) {
                Toast.makeText(ChatActivity.this, message,
                        Toast.LENGTH_SHORT).show();
            }

            else if (message.startsWith("STATUS:Connected to ")) {
                Toast.makeText(ChatActivity.this, message,
                        Toast.LENGTH_SHORT).show();
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
            bluetoothService.addListener(bluetoothListener);
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

        // Load history at startup
        loadMessageHistory();

        // Initialize UI
        tvMessages = findViewById(R.id.tvMessages);
        etMessage = findViewById(R.id.etMessage);
        btnSend = findViewById(R.id.btnSend);
        btnClear = findViewById(R.id.btnClear);
        btnPrev = findViewById(R.id.btnPrev);
        btnNext = findViewById(R.id.btnNext);
        btnClearHistory = findViewById(R.id.btnClearHistory);

        // Bind to BluetoothService
        Intent serviceIntent = new Intent(this, BluetoothService.class);
        bindService(serviceIntent, serviceConnection, BIND_AUTO_CREATE);

        btnClearHistory.setOnClickListener(v -> {
            // Clear in-memory list
            messageHistory.clear();
            historyIndex = -1;

            // Clear SharedPreferences
            SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(this);
            prefs.edit().remove(HISTORY_KEY).apply();

            Toast.makeText(this, "Message history cleared", Toast.LENGTH_SHORT).show();
        });

        btnPrev.setOnClickListener(v -> {
            if (!messageHistory.isEmpty()) {
                if (historyIndex == -1) historyIndex = messageHistory.size() - 1;
                else if (historyIndex > 0) historyIndex--;
                etMessage.setText(messageHistory.get(historyIndex));
                etMessage.setSelection(etMessage.getText().length()); // Move cursor to end
            }
        });

        btnNext.setOnClickListener(v -> {
            if (!messageHistory.isEmpty() && historyIndex != -1) {
                if (historyIndex < messageHistory.size() - 1) {
                    historyIndex++;
                    etMessage.setText(messageHistory.get(historyIndex));
                } else {
                    historyIndex = -1;
                    etMessage.setText("");
                }
                etMessage.setSelection(etMessage.getText().length());
            }
        });

        // Send button
        btnSend.setOnClickListener(v -> {
            if (!isServiceBound) {
                Toast.makeText(this, "Bluetooth service not ready", Toast.LENGTH_SHORT).show();
                return;
            }

            String message = etMessage.getText().toString().trim();
            if (!message.isEmpty() && bluetoothService.getConnectedThread() != null) {
                bluetoothService.getConnectedThread().write(message.getBytes());
                messageHistory.add(message);   // <-- Save message
                saveMessageHistory();
                historyIndex = -1;             // Reset index after sending
                etMessage.setText("");
            } else {
                Toast.makeText(this, "No device connected", Toast.LENGTH_SHORT).show();
            }
        });

        // Clear button
        btnClear.setOnClickListener(v -> etMessage.setText(""));

    }

    @Override
    protected void onStop() {
        super.onStop();
        saveMessageHistory(); // Save when leaving screen
        if (bluetoothService != null) {
            bluetoothService.removeListener(bluetoothListener);
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


    private void saveMessageHistory() {
        if (messageHistory.size() > MAX_HISTORY) {
            // Keep only latest MAX_HISTORY messages
            List<String> latest = messageHistory.subList(
                    messageHistory.size() - MAX_HISTORY, messageHistory.size()
            );
            messageHistory.clear();
            messageHistory.addAll(latest);
        }

        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(this);
        SharedPreferences.Editor editor = prefs.edit();
        JSONArray jsonArray = new JSONArray(messageHistory);
        editor.putString(HISTORY_KEY, jsonArray.toString());
        editor.apply();
    }

    private void loadMessageHistory() {
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(this);
        String jsonString = prefs.getString(HISTORY_KEY, null);
        if (jsonString != null) {
            try {
                JSONArray jsonArray = new JSONArray(jsonString);
                for (int i = 0; i < jsonArray.length(); i++) {
                    messageHistory.add(jsonArray.getString(i));
                }
            } catch (Exception e) {
                Log.d("ChatActivity", "Failed to load message history: " + e.getMessage());
            }
        }
    }

}
