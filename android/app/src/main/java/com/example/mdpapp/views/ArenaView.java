package com.example.mdpapp.views;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.DragEvent;
import android.view.MotionEvent;
import android.view.View;

import com.example.mdpapp.R;
import com.example.mdpapp.models.Arena;
import com.example.mdpapp.models.Obstacle;
import com.example.mdpapp.models.Robot;

public class ArenaView extends View {
    private Arena arena;
    private Paint gridPaint;
    private Paint obstaclePaint;
    private Paint textPaint;
    private Paint robotPaint;

    private Obstacle selectedObstacle;

    private Bitmap robotN, robotS, robotE, robotW;

    public interface OnObstacleChangeListener {
        void onObstacleAdded(Obstacle obstacle);
        void onObstacleRemoved(int obstacleId);
        void onObstacleMoved(Obstacle obstacle);
    }

    public OnObstacleChangeListener getOnObstacleChangeListener() {
        return obstacleChangeListener;
    }

    private OnObstacleChangeListener obstacleChangeListener;

    public void setOnObstacleChangeListener(OnObstacleChangeListener listener) {
        this.obstacleChangeListener = listener;
    }

    public ArenaView(Context context, Arena arena) {
        super(context);
        this.arena = arena;
        init();
    }

    public ArenaView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        gridPaint = new Paint();
        gridPaint.setColor(Color.GRAY);
        gridPaint.setStyle(Paint.Style.STROKE);

        obstaclePaint = new Paint();
        obstaclePaint.setColor(Color.DKGRAY);

        textPaint = new Paint();
        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(30); // adjust font size

        robotPaint = new Paint();
        robotPaint.setColor(Color.RED);

        robotN = BitmapFactory.decodeResource(getResources(), R.drawable.robot_n);
        robotS = BitmapFactory.decodeResource(getResources(), R.drawable.robot_s);
        robotE = BitmapFactory.decodeResource(getResources(), R.drawable.robot_e);
        robotW = BitmapFactory.decodeResource(getResources(), R.drawable.robot_w);

        setOnDragListener((v, event) -> {
            switch (event.getAction()) {
                case DragEvent.ACTION_DRAG_STARTED:
                    return true;

                case DragEvent.ACTION_DRAG_LOCATION:
                    return true;

                case DragEvent.ACTION_DROP:
                    Object localState = event.getLocalState();
                    if (localState instanceof Obstacle) {
                        Obstacle dropped = (Obstacle) localState;

                        // Snap to grid
                        int cols = arena.getWidth();
                        int rows = arena.getHeight();
                        float cellWidth = getWidth() / (float) cols;
                        float cellHeight = getHeight() / (float) rows;

                        int x = Math.round(event.getX() / cellWidth);
                        int y = Math.round(event.getY() / cellHeight);

                        x = Math.max(0, Math.min(x, cols - 1));
                        y = Math.max(0, Math.min(y, rows - 1));

                        dropped.setX(x);
                        dropped.setY(y);
                        arena.addObstacle(dropped);
                        invalidate();

                        if (obstaclePlacedListener != null) {
                            obstaclePlacedListener.onObstaclePlaced(dropped);
                        }
                    }
                    return true;

                default:
                    return true;
            }
        });

    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        drawGrid(canvas);
        drawObstacles(canvas);
        drawRobot(canvas);
    }

    private void drawGrid(Canvas canvas) {
        int rows = arena.getHeight();
        int cols = arena.getWidth();

        float cellWidth = getWidth() / (float) cols;
        float cellHeight = getHeight() / (float) rows;

        for (int i = 0; i <= rows; i++) {
            canvas.drawLine(0, i * cellHeight, getWidth(), i * cellHeight, gridPaint);
        }
        for (int j = 0; j <= cols; j++) {
            canvas.drawLine(j * cellWidth, 0, j * cellWidth, getHeight(), gridPaint);
        }
    }

    private float dragOffsetX = 0;
    private float dragOffsetY = 0;

    private void drawObstacles(Canvas canvas) {
        int rows = arena.getHeight();
        int cols = arena.getWidth();
        float cellWidth = getWidth() / (float) cols;
        float cellHeight = getHeight() / (float) rows;

        for (Obstacle o : arena.getObstacles()) {
            float left = o.getX() * cellWidth;
            float top = o.getY() * cellHeight;
            float right = left + cellWidth;
            float bottom = top + cellHeight;

            float drawLeft = left;
            float drawTop = top;
            float drawRight = right;
            float drawBottom = bottom;

            // Enlarge if this obstacle is being dragged
            if (o == selectedObstacle) {
                float extra = 20; // pixels to enlarge
                drawLeft = left - extra / 2;
                drawTop = top - extra / 2;
                drawRight = right + extra / 2;
                drawBottom = bottom + extra / 2;
            }

            // Draw actual obstacle
            canvas.drawRect(drawLeft, drawTop, drawRight, drawBottom, obstaclePaint);

            // Draw obstacle number in the center
            float textX = drawLeft + (drawRight - drawLeft) / 4;
            float textY = drawTop + (drawBottom - drawTop) / 2;
            canvas.drawText(String.valueOf(o.getId()), textX, textY, textPaint);
        }
    }

    private void drawRobot(Canvas canvas) {
        Robot r = arena.getRobot();
        if (r == null) return;

        int rows = arena.getHeight();
        int cols = arena.getWidth();
        float cellWidth = getWidth() / (float) cols;
        float cellHeight = getHeight() / (float) rows;

        float left = r.getX() * cellWidth;
        float top = r.getY() * cellHeight;

        Bitmap robotBitmap;
        switch (r.getFacing()) {
            case N:
                robotBitmap = robotN;
                break;
            case S:
                robotBitmap = robotS;
                break;
            case E:
                robotBitmap = robotE;
                break;
            case W:
                robotBitmap = robotW;
                break;
            default:
                robotBitmap = robotN;
                break;
        }

        // Scale the bitmap to fit the cell
        Bitmap scaledBitmap = Bitmap.createScaledBitmap(robotBitmap, (int) cellWidth, (int) cellHeight, false);
        canvas.drawBitmap(scaledBitmap, left, top, null);
    }

    public void setArena(Arena arena) {
        this.arena = arena;
        invalidate(); // Redraw the view
    }

    public interface OnObstaclePlacedListener {
        void onObstaclePlaced(Obstacle obstacle);
    }

    private OnObstaclePlacedListener obstaclePlacedListener;

    public void setOnObstaclePlacedListener(OnObstaclePlacedListener listener) {
        this.obstaclePlacedListener = listener;
    }

    public void setSelectedObstacle(Obstacle o) {
        this.selectedObstacle = o;
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        int rows = arena.getHeight();
        int cols = arena.getWidth();
        float cellWidth = getWidth() / (float) cols;
        float cellHeight = getHeight() / (float) rows;

        float touchX = event.getX();
        float touchY = event.getY();

        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
                // Check if touching an existing obstacle
                selectedObstacle = null;
                for (Obstacle o : arena.getObstacles()) {
                    float left = o.getX() * cellWidth;
                    float top = o.getY() * cellHeight;
                    float right = left + cellWidth;
                    float bottom = top + cellHeight;

                    if (touchX >= left && touchX <= right && touchY >= top && touchY <= bottom) {
                        selectedObstacle = o; // pick obstacle to drag

                        // Offset between finger and top-left of obstacle
                        dragOffsetX = touchX - left;
                        dragOffsetY = touchY - top;
                        invalidate();
                        break;
                    }
                }
                break;

            case MotionEvent.ACTION_MOVE:
                if (selectedObstacle != null) {
                    // Move obstacle with finger, accounting for drag offset
                    float newX = (touchX - dragOffsetX + cellWidth / 2) / cellWidth;
                    float newY = (touchY - dragOffsetY + cellHeight / 2) / cellHeight;

                    selectedObstacle.setX(Math.round(newX));
                    selectedObstacle.setY(Math.round(newY));
                    invalidate();
                }
                break;

            case MotionEvent.ACTION_UP:
                dragOffsetX = 0;
                dragOffsetY = 0;
                if (selectedObstacle != null) {
                    // Snap to grid
                    int snappedX = Math.round(touchX / cellWidth);
                    int snappedY = Math.round(touchY / cellHeight);

                    boolean wasInArena = arena.getObstacles().contains(selectedObstacle);

                    // Remove if outside arena
                    if (snappedX < 0 || snappedX >= cols || snappedY < 0 || snappedY >= rows) {
                        if(wasInArena) {
                            arena.removeObstacle(selectedObstacle);
                            if (obstacleChangeListener != null) {
                                obstacleChangeListener.onObstacleRemoved(selectedObstacle.getId());
                            }
                        }
                    } else {
                        selectedObstacle.setX(snappedX);
                        selectedObstacle.setY(snappedY);

                        if (!wasInArena) {
                            arena.addObstacle(selectedObstacle);
                            if (obstacleChangeListener != null) {
                                obstacleChangeListener.onObstacleAdded(selectedObstacle);
                            }
                        } else {
                            if (obstacleChangeListener != null) {
                                obstacleChangeListener.onObstacleMoved(selectedObstacle);
                            }
                        }
                    }

                    selectedObstacle = null; // done dragging
                    invalidate();
                }
                break;
        }

        return true;
    }


}

