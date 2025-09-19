package com.example.mdpapp.models;
public class Robot {
    public enum Direction {N, S, E, W}
    private int x, y;
    private Direction facing;

    public Robot(int x, int y, Direction facing) {
        this.x = x;
        this.y = y;
        this.facing = facing;
    }

    // Getters and setters
    public int getX() { return x; }
    public int getY() { return y; }
    public Direction getFacing() { return facing; }
    public void setX(int x) { this.x = x; }
    public void setY(int y) { this.y = y; }
    public void setFacing(Direction facing) { this.facing = facing; }

    public void moveForward() {
        switch (facing) {
            case N: y--; break;
            case S: y++; break;
            case E: x++; break;
            case W: x--; break;
        }
    }

    public void moveBackward() {
        switch (facing) {
            case N: y++; break;
            case S: y--; break;
            case E: x--; break;
            case W: x++; break;
        }
    }

    public void turnLeft() {
        switch (facing) {
            case N: facing = Direction.W; break;
            case W: facing = Direction.S; break;
            case S: facing = Direction.E; break;
            case E: facing = Direction.N; break;
        }
    }

    public void turnRight() {
        switch (facing) {
            case N: facing = Direction.E; break;
            case E: facing = Direction.S; break;
            case S: facing = Direction.W; break;
            case W: facing = Direction.N; break;
        }
    }
}