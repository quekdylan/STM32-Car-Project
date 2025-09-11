package com.example.mdpapp.models;
public class Obstacle {
    private int id;
    private int x;
    private int y;

    public Obstacle(int id, int x, int y) {
        this.id = id;
        this.x = x;
        this.y = y;
    }

    // Getters and setters
    public int getId() { return id; }
    public int getX() { return x; }
    public int getY() { return y; }
    public void setX(int x) { this.x = x; }
    public void setY(int y) { this.y = y; }
}
