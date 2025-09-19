package com.example.mdpapp.activities;

import static java.lang.Package.getPackage;

import android.annotation.SuppressLint;
import android.os.Bundle;
import android.graphics.Point;
import android.graphics.Canvas;
import android.graphics.Bitmap;

import androidx.appcompat.app.AppCompatActivity;

import com.example.mdpapp.R;
import com.example.mdpapp.models.Arena;
import com.example.mdpapp.models.Obstacle;
import com.example.mdpapp.models.Robot;
import com.example.mdpapp.services.BluetoothService;
import com.example.mdpapp.views.ArenaView;

import android.view.MotionEvent;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.Spinner;
import android.widget.TextView;
import android.content.ComponentName;
import android.content.ServiceConnection;
import android.content.Intent;
import android.os.IBinder;
import android.widget.Toast;


public class ArenaActivity extends AppCompatActivity {
    private Arena arena;
    private ArenaView arenaView;
    private TextView tvRobotCoord, tvRobotDir;

    private EditText etCount;
    private Button btnGenerateMenu, btnForward, btnReverse, btnLeft, btnRight;
    private Spinner spinner;
    private ImageView ivPreview;

    private BluetoothService bluetoothService;
    private boolean isServiceBound = false;

    private BluetoothService.OnMessageReceivedListener bluetoothListener = message -> {
        runOnUiThread(() -> {
            Toast.makeText(ArenaActivity.this, message,
                    Toast.LENGTH_SHORT).show();
        });
        if (message.startsWith("TARGET,")) {
            String[] parts = message.split(",");
            if (parts.length == 3) {
                int obstacleNumber = Integer.parseInt(parts[1].trim());
                int targetId = Integer.parseInt(parts[2].trim());

                runOnUiThread(() -> {
                    Obstacle obstacle = arena.getObstacleById(obstacleNumber);

                    if (obstacle != null) {
                        // Set or unset targetId
                        obstacle.setTargetId(targetId < 0 ? -1 : targetId);
                        arenaView.invalidate(); // Redraw
                    } else {
                        // Obstacle not yet placed
                        Toast.makeText(ArenaActivity.this,
                                "Warning: Target received for obstacle #" + obstacleNumber +
                                        " which is not yet placed.",
                                Toast.LENGTH_SHORT).show();
                    }
                });
            }
        }

        else if (message.startsWith("ROBOT,")){
            String[] parts = message.split(",");
            if(parts.length==4){
                int x = Integer.parseInt(parts[1].trim());
                int y = Integer.parseInt(parts[2].trim());
                Robot.Direction dir = Robot.Direction.valueOf(parts[3].trim());

                if(arena == null || x < 0 || x >= arena.getWidth() || y < 0 || y >= arena.getHeight()){
                    runOnUiThread(() -> Toast.makeText(ArenaActivity.this, "INVALID ROBOT COMMAND! OUT OF BOUNDS!",
                            Toast.LENGTH_SHORT).show());
                    return;
                }

                runOnUiThread(() -> {
                    Robot robot = arena.getRobot();
                    if(robot!=null){
                        robot.setX(x);
                        robot.setY(y);
                        robot.setFacing(dir);
                    }else{
                        robot = new Robot(x, y, dir);
                        arena.setRobot(robot);
                    }

                    arenaView.invalidate();
                    tvRobotCoord.setText("(" + robot.getX() + "," + robot.getY() + ")");
                    tvRobotDir.setText(robot.getFacing().name());
                });

            }
        }
    };

        private final ServiceConnection connection = new ServiceConnection() {
            @Override
            public void onServiceConnected(ComponentName name, IBinder service) {
                BluetoothService.LocalBinder binder = (BluetoothService.LocalBinder) service;
                bluetoothService = binder.getService();
                isServiceBound = true;
                bluetoothService.addListener(bluetoothListener);
                bluetoothService.startAcceptThread();
            }

            @Override
            public void onServiceDisconnected(ComponentName name) {
                isServiceBound = false;
            }
        };

        @Override
        protected void onCreate (Bundle savedInstanceState){
            super.onCreate(savedInstanceState);
            setContentView(R.layout.activity_arena);

            Intent intent = new Intent(this, BluetoothService.class);
            bindService(intent, connection, BIND_AUTO_CREATE);

            tvRobotCoord = findViewById(R.id.tvRobotCoord);
            tvRobotDir = findViewById(R.id.tvRobotDir);
            etCount = findViewById(R.id.etObstacleCount);
            btnGenerateMenu = findViewById(R.id.btnGenerateMenu);
            spinner = findViewById(R.id.spObstacleSpinner);
            ivPreview = findViewById(R.id.ivObstaclePreview);
            arenaView = findViewById(R.id.arenaView);
            btnForward = findViewById(R.id.btnForward);
            btnReverse = findViewById(R.id.btnReverse);
            btnLeft = findViewById(R.id.btnLeft);
            btnRight = findViewById(R.id.btnRight);

            btnGenerateMenu.setOnClickListener(v -> {
                String input = etCount.getText().toString();
                if (input.isEmpty()) return;
                int n = Integer.parseInt(input);

                // Populate spinner with obstacle IDs
                String[] items = new String[n];
                for (int i = 0; i < n; i++) items[i] = String.valueOf(i + 1);

                ArrayAdapter<String> adapter = new ArrayAdapter<>(this,
                        android.R.layout.simple_spinner_item, items);
                adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
                spinner.setAdapter(adapter);
            });

            btnForward.setOnClickListener(v -> sendMovementCommand("f"));
            btnReverse.setOnClickListener(v -> sendMovementCommand("r"));
            btnLeft.setOnClickListener(v -> sendMovementCommand("tl"));
            btnRight.setOnClickListener(v -> sendMovementCommand("tr"));

            spinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
                @Override
                public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                    int obstacleId = position + 1;

                    // Set preview ImageView
                    String drawableName = "obstacle_" + obstacleId;
                    int resId = getResources().getIdentifier(
                            "obstacle_" + obstacleId,
                            "drawable",
                            getPackageName()
                    );

                    ivPreview.setImageResource(resId);
                    ivPreview.setTag(obstacleId);
                }

                @Override
                public void onNothingSelected(AdapterView<?> parent) {
                }
            });

            ivPreview.setOnLongClickListener(v -> {
                int id = (int) v.getTag();

                // Check if obstacle with this ID already exists in arena
                Obstacle existing = arena.getObstacleById(id);

                if (existing != null) {
                    // Pick up existing obstacle
                    arena.removeObstacle(existing);
                    arenaView.invalidate();

                    // Notify listener
                    if (arenaView.getOnObstacleChangeListener() != null) {
                        arenaView.getOnObstacleChangeListener().onObstacleRemoved(existing.getId());
                    }
                }

                Obstacle obstacleToDrag = existing != null ? existing : new Obstacle(id, -1, -1);

                // Start drag, passing the obstacle as local state
                View.DragShadowBuilder shadow = new View.DragShadowBuilder(v) {
                    @Override
                    public void onProvideShadowMetrics(Point size, Point touch) {
                        // Make the shadow bigger than the view
                        int width = (int) (v.getWidth() * 1.5f);
                        int height = (int) (v.getHeight() * 1.5f);
                        size.set(width, height);

                        // Position the touch point at the center of the shadow
                        touch.set(width / 2, height / 2);
                    }

                    @Override
                    public void onDrawShadow(Canvas canvas) {
                        // Draw the scaled bitmap of the view
                        Bitmap bitmap = Bitmap.createBitmap(v.getWidth(), v.getHeight(), Bitmap.Config.ARGB_8888);
                        Canvas c = new Canvas(bitmap);
                        v.draw(c);
                        Bitmap scaled = Bitmap.createScaledBitmap(bitmap, (int) (v.getWidth() * 1.5f), (int) (v.getHeight() * 1.5f), true);
                        canvas.drawBitmap(scaled, 0, 0, null);
                    }
                };

                v.startDragAndDrop(null, shadow, obstacleToDrag, 0);
                return true;
            });

            arenaView.setOnObstaclePlacedListener(obstacle -> {
                sendBluetoothMessage("OBSTACLE_ADD," + obstacle.getId() + "," + obstacle.getX() + "," + obstacle.getY());
            });

            arena = new Arena(10, 10); // 10x10 grid
            arenaView.setArena(arena);

            arenaView.setOnObstacleChangeListener(new ArenaView.OnObstacleChangeListener() {
                @Override
                public void onObstacleAdded(Obstacle obstacle) {
                    sendBluetoothMessage("OBSTACLE_ADD," + obstacle.getId() + "," + obstacle.getX() + "," + obstacle.getY());
                }

                @Override
                public void onObstacleRemoved(int obstacleId) {
                    sendBluetoothMessage("OBSTACLE_REMOVE," + obstacleId);
                }

                @Override
                public void onObstacleMoved(Obstacle obstacle) {
                    sendBluetoothMessage("OBSTACLE_MOVE," + obstacle.getId() + "," + obstacle.getX() + "," + obstacle.getY());
                }

                @Override
                public void onObstacleFaceAdded(Obstacle obstacle) {
                    sendBluetoothMessage("OBSTACLE_FACE," + obstacle.getId() + "," + obstacle.getTargetFace().name());
                }

                @Override
                public void onObstacleFaceRemoved(Obstacle obstacle) {
                    sendBluetoothMessage("OBSTACLE_FACE_REMOVE," + obstacle.getId());
                }

            });

        }

        @Override
        protected void onDestroy () {
            super.onDestroy();
            if (isServiceBound) {
                if (bluetoothService != null) {
                    bluetoothService.removeListener(bluetoothListener);
                }
                unbindService(connection);
                isServiceBound = false;
            }
        }

        private void sendBluetoothMessage (String message){
            if (bluetoothService != null) {
                BluetoothService.ConnectedThread thread = bluetoothService.getConnectedThread();
                if (thread != null) {
                    thread.write(message.getBytes());
                }
            }
        }

        private void sendMovementCommand(String command) {
            if (!isServiceBound) {
                Toast.makeText(this, "Bluetooth service not ready", Toast.LENGTH_SHORT).show();
                return;
            }

            // Update robot locally before sending command
            Robot robot = arena.getRobot();
            if (robot != null) {
                switch (command) {
                    case "f":
                        robot.moveForward();
                        break;
                    case "r":
                        robot.moveBackward();
                        break;
                    case "tl":
                        robot.turnLeft();
                        break;
                    case "tr":
                        robot.turnRight();
                        break;
                }
                arenaView.invalidate(); // Redraw robot immediately
                tvRobotCoord.setText("(" + robot.getX() + "," + robot.getY() + ")");
                tvRobotDir.setText(robot.getFacing().name());
            }

            if (bluetoothService.getConnectedThread() != null) {
                bluetoothService.getConnectedThread().write(command.getBytes());
            } else {
                Toast.makeText(this, "No device connected", Toast.LENGTH_SHORT).show();
            }
        }
    }
