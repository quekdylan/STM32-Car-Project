package com.example.mdpapp.views;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.view.DragEvent;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.View;

import androidx.constraintlayout.widget.ConstraintSet;

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

    private Obstacle selectedObstacle, selectedForSide;

    private GestureDetector gestureDetector;

    private Bitmap robotN, robotS, robotE, robotW;
    private Bitmap arrowUp, arrowDown, arrowLeft, arrowRight;

    public interface OnObstacleChangeListener {
        void onObstacleAdded(Obstacle obstacle);

        void onObstacleRemoved(int obstacleId);

        void onObstacleMoved(Obstacle obstacle);

        void onObstacleFaceAdded(Obstacle obstacle);

        void onObstacleFaceRemoved(Obstacle obstacle);
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

    @Override
    protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        super.onMeasure(widthMeasureSpec, heightMeasureSpec);
        int size = Math.min(getMeasuredWidth(), getMeasuredHeight());
        setMeasuredDimension(size, size); // make the view square
    }

    private void init() {
        gridPaint = new Paint();
        gridPaint.setColor(Color.GRAY);
        gridPaint.setStyle(Paint.Style.STROKE);

        obstaclePaint = new Paint();
        obstaclePaint.setColor(Color.DKGRAY);

        textPaint = new Paint();
        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(10); // adjust font size

        robotPaint = new Paint();
        robotPaint.setColor(Color.RED);

        robotN = BitmapFactory.decodeResource(getResources(), R.drawable.robot_n);
        robotS = BitmapFactory.decodeResource(getResources(), R.drawable.robot_s);
        robotE = BitmapFactory.decodeResource(getResources(), R.drawable.robot_e);
        robotW = BitmapFactory.decodeResource(getResources(), R.drawable.robot_w);

        arrowUp = BitmapFactory.decodeResource(getResources(), R.drawable.arrow_up);
        arrowDown = BitmapFactory.decodeResource(getResources(), R.drawable.arrow_down);
        arrowLeft = BitmapFactory.decodeResource(getResources(), R.drawable.arrow_left);
        arrowRight = BitmapFactory.decodeResource(getResources(), R.drawable.arrow_right);

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
                        float cellSize = Math.min(getWidth() / (float) cols, getHeight() / (float) rows);

                        int x = Math.round(event.getX() / cellSize);
                        int y = Math.round(event.getY() / cellSize);

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

        gestureDetector = new GestureDetector(getContext(), new GestureDetector.SimpleOnGestureListener() {
            @Override
            public boolean onDoubleTap(MotionEvent e) {
                handleDoubleTap(e.getX(), e.getY());
                return true;
            }

            @Override
            public void onLongPress(MotionEvent e) {
                handleLongPress(e.getX(), e.getY());
            }
        });

    }

    private void handleDoubleTap(float x, float y) {
        int rows = arena.getHeight();
        int cols = arena.getWidth();
        float cellSize = Math.min(getWidth() / (float) cols, getHeight() / (float) rows);

        for (Obstacle o : arena.getObstacles()) {
            float left = o.getX() * cellSize;
            float top = o.getY() * cellSize;
            float right = left + cellSize;
            float bottom = top + cellSize;

            if (x >= left && x <= right && y >= top && y <= bottom) {
                selectedForSide = o;  // mark this obstacle for side selection
                invalidate();          // redraw to show arrows
                break;
            }
        }
    }

    // Triggered when user long-presses an obstacle
    private void handleLongPress(float x, float y) {
        int rows = arena.getHeight();
        int cols = arena.getWidth();
        float cellSize = Math.min(getWidth() / (float) cols, getHeight() / (float) rows);

        selectedObstacle = null;
        for (Obstacle o : arena.getObstacles()) {
            float left = o.getX() * cellSize;
            float top = o.getY() * cellSize;
            float right = left + cellSize;
            float bottom = top + cellSize;

            if (x >= left && x <= right && y >= top && y <= bottom) {
                selectedObstacle = o;

                // Offset between finger and top-left of obstacle
                dragOffsetX = x - left;
                dragOffsetY = y - top;
                invalidate();
                break;
            }
        }
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
        float cellSize = Math.min(getWidth() / (float) cols, getHeight() / (float) rows);

        for (int i = 0; i <= rows; i++) {
            canvas.drawLine(0, i * cellSize, getWidth(), i * cellSize, gridPaint);
        }
        for (int j = 0; j <= cols; j++) {
            canvas.drawLine(j * cellSize, 0, j * cellSize, getHeight(), gridPaint);
        }
    }

    private float dragOffsetX = 0;
    private float dragOffsetY = 0;

    private void drawObstacles(Canvas canvas) {
        int rows = arena.getHeight();
        int cols = arena.getWidth();
        float cellSize = Math.min(getWidth() / (float) cols, getHeight() / (float) rows);

        for (Obstacle o : arena.getObstacles()) {
            float left = o.getX() * cellSize;
            float top = o.getY() * cellSize;
            float right = left + cellSize;
            float bottom = top + cellSize;

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

            if (o == selectedForSide) {
                int arrowSize = 50; // adjust size
                Bitmap up = Bitmap.createScaledBitmap(arrowUp, arrowSize, arrowSize, false);
                Bitmap down = Bitmap.createScaledBitmap(arrowDown, arrowSize, arrowSize, false);
                Bitmap leftBmp = Bitmap.createScaledBitmap(arrowLeft, arrowSize, arrowSize, false);
                Bitmap rightBmp = Bitmap.createScaledBitmap(arrowRight, arrowSize, arrowSize, false);

                canvas.drawBitmap(up, (drawLeft + drawRight) / 2 - arrowSize / 2, drawTop - arrowSize - 10, null);
                canvas.drawBitmap(down, (drawLeft + drawRight) / 2 - arrowSize / 2, drawBottom + 10, null);
                canvas.drawBitmap(leftBmp, drawLeft - arrowSize - 10, (drawTop + drawBottom) / 2 - arrowSize / 2, null);
                canvas.drawBitmap(rightBmp, drawRight + 10, (drawTop + drawBottom) / 2 - arrowSize / 2, null);
            }

            // Draw actual obstacle
            canvas.drawRect(drawLeft, drawTop, drawRight, drawBottom, obstaclePaint);

            if (o.getTargetFace() != null) {
                Paint facePaint = new Paint();
                facePaint.setColor(Color.YELLOW);

                switch (o.getTargetFace()) {
                    case N:
                        // Top side, full width
                        canvas.drawRect(drawLeft, drawTop, drawRight, drawTop + 10, facePaint);
                        break;
                    case S:
                        // Bottom side, full width
                        canvas.drawRect(drawLeft, drawBottom - 10, drawRight, drawBottom, facePaint);
                        break;
                    case W:
                        // Left side, full height
                        canvas.drawRect(drawLeft, drawTop, drawLeft + 10, drawBottom, facePaint);
                        break;
                    case E:
                        // Right side, full height
                        canvas.drawRect(drawRight - 10, drawTop, drawRight, drawBottom, facePaint);
                        break;
                }
            }

            // Draw obstacle number in the center
            String text = o.getTargetId() > 0 ? String.valueOf(o.getTargetId()) : String.valueOf(o.getId());
            if (o.getTargetId() > 0) {
                textPaint.setTextSize(40); // larger font for target
            } else {
                textPaint.setTextSize(10); // default small font
            }
            float textX = drawLeft + (drawRight - drawLeft) / 2 - textPaint.measureText(text) / 2;
            float textY = drawTop + (drawBottom - drawTop) / 2 + textPaint.getTextSize() / 2;
            canvas.drawText(text, textX, textY, textPaint);
        }
    }

    private void drawRobot(Canvas canvas) {
        Robot r = arena.getRobot();
        if (r == null) return;

        int rows = arena.getHeight();
        int cols = arena.getWidth();
        float cellSize = Math.min(getWidth() / (float) cols, getHeight() / (float) rows);

        float left = r.getX() * cellSize;
        float top = r.getY() * cellSize;

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
        Bitmap scaledBitmap = Bitmap.createScaledBitmap(robotBitmap, (int) cellSize, (int) cellSize, false);
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
        gestureDetector.onTouchEvent(event); // Handle gestures

        if (selectedObstacle == null && selectedForSide == null) return true;


        int rows = arena.getHeight();
        int cols = arena.getWidth();
        float cellSize = Math.min(getWidth() / (float) cols, getHeight() / (float) rows);

        // Change surface for side selection after select obstacle and
        if (selectedForSide != null && event.getAction() == MotionEvent.ACTION_DOWN) {
            float drawLeft = selectedForSide.getX() * cellSize;
            float drawTop = selectedForSide.getY() * cellSize;
            float drawRight = drawLeft + cellSize;
            float drawBottom = drawTop + cellSize;

            int arrowSize = 50; // adjust size
            float centerX = (drawLeft + drawRight) / 2 - arrowSize / 2;
            float centerY = (drawTop + drawBottom) / 2 - arrowSize / 2;

            RectF upRect = new RectF(centerX, drawTop - arrowSize - 10, centerX + arrowSize, drawTop - 10);
            RectF downRect = new RectF(centerX, drawBottom + 10, centerX + arrowSize, drawBottom + 10 + arrowSize);
            RectF leftRect = new RectF(drawLeft - arrowSize - 10, centerY, drawLeft - 10, centerY + arrowSize);
            RectF rightRect = new RectF(drawRight + 10, centerY, drawRight + 10 + arrowSize, centerY + arrowSize);

            // Check which arrow was tapped
            Obstacle.Direction prevFace = selectedForSide.getTargetFace();
            if (upRect.contains(event.getX(), event.getY())) {
                selectedForSide.setTargetFace(prevFace == Obstacle.Direction.N ? null : Obstacle.Direction.N);
            } else if (downRect.contains(event.getX(), event.getY())) {
                selectedForSide.setTargetFace(prevFace == Obstacle.Direction.S ? null : Obstacle.Direction.S);
            } else if (leftRect.contains(event.getX(), event.getY())) {
                selectedForSide.setTargetFace(prevFace == Obstacle.Direction.W ? null : Obstacle.Direction.W);
            } else if (rightRect.contains(event.getX(), event.getY())) {
                selectedForSide.setTargetFace(prevFace == Obstacle.Direction.E ? null : Obstacle.Direction.E);
            } else {
                return true; // tap outside arrows
            }


            // Notify listener (to broadcast via Bluetooth)
            if (obstacleChangeListener != null) {
                if (selectedForSide.getTargetFace() != null) {
                    obstacleChangeListener.onObstacleFaceAdded(selectedForSide);
                } else {
                    obstacleChangeListener.onObstacleFaceRemoved(selectedForSide);
                }
            }

            selectedForSide = null; // done selecting side

            // Redraw obstacle to reflect direction selection
            invalidate();

            return true; // consume touch
        }

        // Handle dragging
        float touchX = event.getX();
        float touchY = event.getY();

        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
                // Check if touching an existing obstacle
                selectedObstacle = null;
                for (Obstacle o : arena.getObstacles()) {
                    float left = o.getX() * cellSize;
                    float top = o.getY() * cellSize;
                    float right = left + cellSize;
                    float bottom = top + cellSize;

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
                    float newX = (touchX - dragOffsetX + cellSize / 2) / cellSize;
                    float newY = (touchY - dragOffsetY + cellSize / 2) / cellSize;

                    selectedObstacle.setX(Math.round(newX));
                    selectedObstacle.setY(Math.round(newY));
                    invalidate();
                }
                break;

            case MotionEvent.ACTION_UP:
                if (selectedObstacle != null) {
                    dragOffsetX = 0;
                    dragOffsetY = 0;

                    // Snap to grid
                    int snappedX = Math.round(touchX / cellSize);
                    int snappedY = Math.round(touchY / cellSize);

                    boolean wasInArena = arena.getObstacles().contains(selectedObstacle);

                    // Remove if outside arena
                    if (snappedX < 0 || snappedX >= cols || snappedY < 0 || snappedY >= rows) {
                        if (wasInArena) {
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

