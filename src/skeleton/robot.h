#ifndef ROBOT_H
#define ROBOT_H

#include "bone.h"

// controls bones and target navigation
// don't propagate scale in the matrix

class Robot {
private:
    Bone* root;

    std::vector<Select*> selects;
    int selectedIndex = 0;

public:
    Robot();
    ~Robot();

    void draw(MatrixStack& MV, GLint aPosLoc, GLint aNorLoc, const GLint MVLoc);

    // move through selects
    void iterate(int i);
};

#endif // ROBOT_H