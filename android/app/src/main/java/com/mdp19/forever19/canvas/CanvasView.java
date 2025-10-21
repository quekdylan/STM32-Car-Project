package com.mdp19.forever19.canvas;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

import androidx.annotation.NonNull;

import com.mdp19.forever19.Facing;

public class CanvasView extends View {
    private final String TAG = "CanvasView";
    private int cellSize;  // Calculated dynamically
    private int offsetX, offsetY; // To center the grid
    private final Paint gridPaint = new Paint();
    private final Paint textPaint = new Paint();
    private final Paint idPaint = new Paint();
    private final Paint facingPaint = new Paint();
    private final Paint targetPaint = new Paint();
    private final Paint startRegionPaint = new Paint();
    private final Paint sandLight = new Paint();
    private final Paint sandDark  = new Paint();
    private final Paint shadowPaint = new Paint();
    private final Paint bushBasePaint   = new Paint();
    private final Paint bushLightPaint  = new Paint();
    private final Paint bushShadowPaint = new Paint();
    private final Paint padFillPaint = new Paint();
    private final Paint padRingPaint = new Paint();
    private final Paint padOuterPaint = new Paint();
    private int highlightX = -1, highlightY = -1;
    private final Paint highlightPaint = new Paint();
    private boolean showFooter = false;
    private int footerX = -1, footerY = -1;
    private int footerBottomMarginPx = 0;
    private final Paint footerBgPaint = new Paint();
    private final Paint footerTextPaint = new Paint();
    private Facing footerFacing = Facing.NORTH;


    private Grid grid;

    public CanvasView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    @Override
    public boolean performClick() {
        return super.performClick();
    }

    private void init() {
        // Grid styling
        gridPaint.setColor(Color.BLACK);
        gridPaint.setStrokeWidth(2);
        gridPaint.setStyle(Paint.Style.STROKE);

        // Label text styling
        textPaint.setColor(Color.BLACK);
        textPaint.setTextSize(20);  // Adjust for readability
        textPaint.setTextAlign(Paint.Align.CENTER);
        textPaint.setFakeBoldText(true);

        // ID text styling (obstacle IDs)
        idPaint.setColor(Color.WHITE);
        idPaint.setTextAlign(Paint.Align.CENTER);
        idPaint.setFakeBoldText(false);
        idPaint.setTextSize(16);

        // Target styling
        targetPaint.setColor(Color.GREEN);
        targetPaint.setTextAlign(Paint.Align.CENTER);
        targetPaint.setFakeBoldText(true);
        targetPaint.setTextSize(24);

        // Facing indicator styling (Orange Strip)
        facingPaint.setColor(Color.rgb(32, 78, 74));
        facingPaint.setStyle(Paint.Style.FILL);

        // Initialize startRegionPaint
        startRegionPaint.setColor(Color.WHITE);
        startRegionPaint.setStrokeWidth(4);  // Make it slightly thicker
        startRegionPaint.setStyle(Paint.Style.STROKE);

        // Highlight styling
        highlightPaint.setStyle(Paint.Style.FILL);
        highlightPaint.setColor(0x40A52A2A);

        // Footer background (semi-transparent)
        footerBgPaint.setStyle(Paint.Style.FILL);
        footerBgPaint.setColor(0xAA000000); // ~66% black

        // Footer text
        footerTextPaint.setColor(Color.WHITE);
        footerTextPaint.setTextAlign(Paint.Align.CENTER);
        footerTextPaint.setFakeBoldText(true);
        footerTextPaint.setTextSize(32);

        // sand tones
        sandLight.setColor(Color.rgb(247, 207, 144)); // light sand
        sandDark.setColor(Color.rgb(233, 184, 117));  // darker sand

        // Bush colors
        bushBasePaint.setColor(Color.rgb(46, 160, 67));     // main green
        bushBasePaint.setStyle(Paint.Style.FILL);

        bushLightPaint.setColor(Color.rgb(88, 200, 92));    // light leaf tips
        bushLightPaint.setStyle(Paint.Style.FILL);

        bushShadowPaint.setColor(Color.argb(60, 0, 0, 0));  // subtle drop shadow
        bushShadowPaint.setStyle(Paint.Style.FILL);

        // Spawn pad
        padFillPaint.setStyle(Paint.Style.FILL);
        padFillPaint.setColor(Color.argb(120, 66, 144, 245));  // translucent blue

        padRingPaint.setStyle(Paint.Style.STROKE);
        padRingPaint.setStrokeWidth(6f);
        padRingPaint.setColor(Color.argb(220, 11, 92, 173));  // solid ring

        padOuterPaint.setStyle(Paint.Style.STROKE);
        padOuterPaint.setStrokeWidth(18f);
        padOuterPaint.setColor(Color.argb(90, 66, 144, 245));  // soft outer halo

        // id text a touch bigger to read on blocks
        idPaint.setTextSize(18);

    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);

        int gridSize = Grid.GRID_SIZE;

        // Compute cell size dynamically
        cellSize = Math.min(w, h) / (gridSize + 2); // +2 for axis labels

        // Offset to keep grid centered
        offsetX = (w - (gridSize * cellSize)) / 2;
        offsetY = (h - (gridSize * cellSize)) / 2;
    }

    @Override
    protected void onDraw(@NonNull Canvas canvas) {
        super.onDraw(canvas);
        drawSandBackground(canvas);
        drawGrid(canvas);
        drawStartRegion(canvas);
        drawAxisLabels(canvas);
        drawRowColHighlight(canvas);
        drawObstacles(canvas);
        if (showFooter) {
            drawCoordFooter(canvas);
        }
    }

    private void drawGrid(Canvas canvas) {
        int gridSize = Grid.GRID_SIZE;
        int gridWidth = gridSize * cellSize;
        int gridHeight = gridSize * cellSize;

        Paint brownPaint = new Paint();
        brownPaint.setColor(Color.rgb(125,75,19));
        brownPaint.setStrokeWidth(2);
        brownPaint.setStyle(Paint.Style.STROKE);

        Paint brownPaint2 = new Paint();
        brownPaint2.setColor(Color.rgb(125,39,18));
        brownPaint2.setStrokeWidth(2);
        brownPaint2.setStyle(Paint.Style.STROKE);

        // Draw vertical grid lines (alternating colors)
        for (int i = 0; i <= gridSize; i++) {
            Paint paintToUse = (i % 2 == 0) ? brownPaint : brownPaint2;
            canvas.drawLine(offsetX + i * cellSize, offsetY, offsetX + i * cellSize, offsetY + gridHeight, paintToUse);
        }

        // Draw horizontal grid lines (alternating colors)
        for (int i = 0; i <= gridSize; i++) {
            Paint paintToUse = (i % 2 == 0) ? brownPaint2 : brownPaint;
            canvas.drawLine(offsetX, offsetY + i * cellSize, offsetX + gridWidth, offsetY + i * cellSize, paintToUse);
        }
    }

    private void drawSandBackground(Canvas canvas) {
        int gridSize = Grid.GRID_SIZE;
        for (int gx = 0; gx < gridSize; gx++) {
            for (int gy = 0; gy < gridSize; gy++) {
                boolean dark = ((gx + gy) % 2 == 0);
                Paint p = dark ? sandDark : sandLight;

                int left   = offsetX + gx * cellSize;
                int top    = offsetY + (gridSize - 1 - gy) * cellSize; // flip y
                int right  = left + cellSize;
                int bottom = top + cellSize;

                canvas.drawRect(left, top, right, bottom, p);
            }
        }
    }

    private void drawStartRegion(Canvas canvas) {
        final int SPAWN_CX = 1;
        final int SPAWN_CY = 1;

        // pixel center of that cell
        float cx = offsetX + (SPAWN_CX + 0.5f) * cellSize;
        float cy = offsetY + (Grid.GRID_SIZE - SPAWN_CY - 0.5f) * cellSize;

        float rOuter = 1.45f * cellSize;
        float rRing  = 1.55f * cellSize;
        float rFill  = 1.35f * cellSize;

        canvas.drawCircle(cx, cy, rOuter, padOuterPaint);
        canvas.drawCircle(cx, cy, rRing,  padRingPaint);
        canvas.drawCircle(cx, cy, rFill,  padFillPaint);
    }

    private void drawAxisLabels(Canvas canvas) {
        int gridSize = Grid.GRID_SIZE;

        // Adjustments for positioning
        float yLabelOffsetY = cellSize / 4f;  // Shift down for Y-axis labels

        // Draw X-axis labels (below the grid)
        for (int x = 0; x < gridSize; x++) {
            if (x == 0) continue; // Skip drawing "0" here to avoid duplication at (0,0)
            canvas.drawText(
                    String.valueOf(x),
                    offsetX + x * cellSize,  // Align with left vertical grid line
                    offsetY + (gridSize * cellSize) + 30,    // Shift below grid
                    textPaint
            );
        }

        // Draw Y-axis labels (left of the grid)
        for (int y = 0; y < gridSize; y++) {
            if (y == 0) continue; // Skip drawing "0" here to avoid duplication at (0,0)
            canvas.drawText(
                    String.valueOf(y),
                    offsetX - 30,  // Shift left of the grid
                    offsetY + (gridSize - y) * cellSize + yLabelOffsetY,  // Align with bottom horizontal grid line
                    textPaint
            );
        }

        // Draw (0,0) label at bottom-left of the grid
        canvas.drawText(
                "0",
                offsetX - 30,  // Align to left of the grid
                offsetY + (gridSize * cellSize) + 30,  // Align to bottom of the grid
                textPaint
        );
    }

    private void drawRowColHighlight(Canvas canvas) {
        if (highlightX < 0 || highlightY < 0) return;

        int gridSize = Grid.GRID_SIZE;
        int gridPxWidth  = gridSize * cellSize;
        int gridPxHeight = gridSize * cellSize;

        // Row rectangle (entire row at Y)
        float rowTop = offsetY + (gridSize - 1 - highlightY) * cellSize; // flip Y once
        canvas.drawRect(
                offsetX,
                rowTop,
                offsetX + gridPxWidth,
                rowTop + cellSize,
                highlightPaint
        );

        // Column rectangle (entire column at X)
        float colLeft = offsetX + highlightX * cellSize;
        canvas.drawRect(
                colLeft,
                offsetY,
                colLeft + cellSize,
                offsetY + gridPxHeight,
                highlightPaint
        );
    }

    private void drawBushRect(Canvas c, int left, int top, int right, int bottom, boolean selected) {
        int inset = Math.max(1, (int)(cellSize * 0.06f));
        float L = left + inset;
        float T = top  + inset;
        float R = right - inset;
        float B = bottom - inset;

        // drop shadow (offset)
        float dx = Math.max(1f, cellSize * 0.04f);
        float dy = Math.max(1f, cellSize * 0.06f);
        c.drawRect(L + dx, T + dy, R + dx, B + dy, bushShadowPaint);

        // base rectangle
        c.drawRect(L, T, R, B, selected ? bushLightPaint : bushBasePaint);
    }

    private void drawObstacles(Canvas canvas) {
        int id = 1;
        for (GridObstacle gridObstacle : grid.getObstacleList()) {
            int gx = gridObstacle.getPosition().getXInt();
            int gy = gridObstacle.getPosition().getYInt();

            int left = offsetX + gx * cellSize;
            int top = offsetY + (Grid.GRID_SIZE - 1 - gy) * cellSize; // flip y
            int right = left + cellSize;
            int bottom = top + cellSize;

            boolean selected = gridObstacle.isSelected();

            // draw bush-style rectangle
            drawBushRect(canvas, left, top, right, bottom, selected);

            // ID/target text (white pops on green)
            float textX = left + (cellSize / 2f);
            float textY = top + (cellSize / 2f) - ((idPaint.descent() + idPaint.ascent()) / 2f);
            if (gridObstacle.getTarget() == null) {
                canvas.drawText(String.valueOf(gridObstacle.getId()), textX, textY, idPaint);
            } else {
                canvas.drawText(gridObstacle.getTarget().getTargetStr(), textX, textY, targetPaint);
            }

            drawFacingIndicator(canvas, gridObstacle.getFacing(), left, top, right, bottom);
        }
    }

    private void drawFacingIndicator(Canvas canvas, Facing facing, int left, int top, int right, int bottom) {
        int stripThickness = cellSize / 6; // Adjust strip size relative to the cell size

        switch (facing) {
            case NORTH:
                canvas.drawRect(left, top, right, top + stripThickness, facingPaint);
                break;
            case EAST:
                canvas.drawRect(right - stripThickness, top, right, bottom, facingPaint);
                break;
            case SOUTH:
                canvas.drawRect(left, bottom - stripThickness, right, bottom, facingPaint);
                break;
            case WEST:
                canvas.drawRect(left, top, left + stripThickness, bottom, facingPaint);
                break;
        }
    }

    private void drawCoordFooter(Canvas c) {
        int w = getWidth();
        int h = getHeight();

        // Bar height ~ 1 cell (with min size for small screens)
        int barH = Math.max((int)(cellSize * 0.9f), 48);

        // Background bar at the very bottom of the view
        float left = 0;
        float right = w;
        float top  = h - footerBottomMarginPx - barH;
        float bottom = h - footerBottomMarginPx;

        c.drawRect(left, top, right, bottom, footerBgPaint);

        // Text "(x,y)" centered in the bar
        String label = "(" + footerX + "," + footerY + ") " + arrowFor(footerFacing);
        float textSize = Math.max(24f, cellSize * 0.45f);
        footerTextPaint.setTextSize(textSize);
        Paint.FontMetrics fm = footerTextPaint.getFontMetrics();
        float textY = top + (barH - fm.bottom - fm.top) / 2f;

        c.drawText(label, w / 2f, textY, footerTextPaint);
    }

    public int getOffsetX() {
        return offsetX;
    }

    public int getOffsetY() {
        return offsetY;
    }

    public int getCellSize() {
        return cellSize;
    }

    public void setGrid(Grid grid) {
        this.grid = grid;
        invalidate(); // Refresh the view
    }

    public void setHighlightCell(int x, int y) {
        this.highlightX = x;
        this.highlightY = y;
        invalidate();
    }
    public void clearHighlight() {
        this.highlightX = -1;
        this.highlightY = -1;
        invalidate();
    }

    public void showFooterCoord(int x, int y, Facing facing) {
        this.footerX = x;
        this.footerY = y;
        this.footerFacing = (facing != null) ? facing : Facing.NORTH;
        this.showFooter = true;
        invalidate();
    }

    private String arrowFor(Facing f) {
        switch (f) {
            case NORTH: return "↑";
            case EAST:  return "→";
            case SOUTH: return "↓";
            case WEST:  return "←";
            default:    return "↑";
        }
    }

    public void hideFooterCoord() {
        this.showFooter = false;
        invalidate();
    }

    public void setFooterBottomMarginDp(int dp) {
        float d = getResources().getDisplayMetrics().density;
        footerBottomMarginPx = (int) (dp * d);
        invalidate();
    }

}