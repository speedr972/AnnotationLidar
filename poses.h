#ifndef POSES_H
#define POSES_H


struct Pose2D{
    int width;
    int height;
    int x;
    int y;
};

struct Pose3D{
    //right now : cube center
    float x;
    float y;
    float z;
    //TODO : wihch one correspond to which axis
    //currently, theta is z rotation
    float theta;
    float psi;
    float phi;
};

#endif // POSES_H
