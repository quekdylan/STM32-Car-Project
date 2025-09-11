package com.example.mdpapp;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

public class ArenaView extends View {
    private int rows = 10;
    private int cols = 10;
    private int[][] cells; // 0 = empty, 1 = obstacle
    private Paint linePaint, obstaclePaint;
    private int cellWidth, cellHeight;

    public ArenaView(Context context) {
        super(context);
        init();
    }

    public ArenaView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        cells = new int[rows][cols];
        linePaint = new Paint();
        linePaint.setColor(Color.BLACK);
        linePaint.setStrokeWidth(2);

        obstaclePaint = new Paint();
        obstaclePaint.setColor(Color.RED); // obstacle color
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        int width = getWidth();
        int height = getHeight();
        cellWidth = width / cols;
        cellHeight = height / rows;

        // Draw grid lines
        for (int i = 0; i <= rows; i++) {
            canvas.drawLine(0, i * cellHeight, width, i * cellHeight, linePaint);
        }
        for (int j = 0; j <= cols; j++) {
            canvas.drawLine(j * cellWidth, 0, j * cellWidth, height, linePaint);
        }

        // Draw obstacles
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                if (cells[r][c] == 1) {
                    canvas.drawRect(c * cellWidth, r * cellHeight,
                            (c + 1) * cellWidth, (r + 1) * cellHeight, obstaclePaint);
                }
            }
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        if (event.getAction() == MotionEvent.ACTION_DOWN) {
            int col = (int) (event.getX() / cellWidth);
            int row = (int) (event.getY() / cellHeight);

            if (row >= 0 && row < rows && col >= 0 && col < cols) {
                // Toggle obstacle on tap
                cells[row][col] = (cells[row][col] == 0) ? 1 : 0;
                invalidate(); // Redraw
            }
        }
        return true;
    }
}

