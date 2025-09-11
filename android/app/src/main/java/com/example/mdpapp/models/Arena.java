package com.example.mdpapp.models;

import java.util.ArrayList;
import java.util.List;

public class Arena {
    private int width, height;
    private List<Obstacle> obstacles;
    private Robot robot;

    public Arena(int width, int height) {
        this.width = width;
        this.height = height;
        obstacles = new ArrayList<>();

    }

    // Getters for width and height
    public int getWidth() { return width; }
    public int getHeight() { return height; }


    public void setRobot(Robot robot) { this.robot = robot; }
    public Robot getRobot() { return robot; }



    public List<Obstacle> getObstacles() { return obstacles; }

    public Obstacle getObstacleById(int id) {
        for (Obstacle o : obstacles) {
            if (o.getId() == id) return o;
        }
        return null;
    }

    public void addObstacle(Obstacle o) { obstacles.add(o); }
    public void removeObstacle(Obstacle o) { obstacles.remove(o); }
}
