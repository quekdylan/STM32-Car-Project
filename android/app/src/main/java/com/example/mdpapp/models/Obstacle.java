package com.example.mdpapp.models;
public class Obstacle {
    public enum Direction {N, S, E, W}
    private int id;
    private int x;
    private int y;
    private Direction targetFace;
    private int targetId = -1; // -1 indicates no target

    public Obstacle(int id, int x, int y) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.targetFace = null;

    }

    // Getters and setters
    public int getId() { return id; }
    public int getX() { return x; }
    public int getY() { return y; }
    public void setX(int x) { this.x = x; }
    public void setY(int y) { this.y = y; }

    public Direction getTargetFace() {return targetFace; }
    public void setTargetFace(Direction targetFace) { this.targetFace = targetFace; }

    public int getTargetId() { return targetId; }
    public void setTargetId(int targetId) { this.targetId = targetId; }

}
