#include "robot.h"
#include "../render/matrix_stack.h"

Robot::Robot() 
{
    root = new Bone(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 1.0f, 1.0f));
}

Robot::~Robot() 
{
    delete root;
}

void Robot::draw(MatrixStack& MV, GLint aPosLoc, GLint aNorLoc, const GLint MVLoc)
{
    root->draw(MV, aPosLoc, aNorLoc, MVLoc);
}

void Robot::iterate(int i) {
    // swap bone selection
    selects[selectedIndex]->setSelected(false);

    // shift
    selectedIndex += i;
    if (selectedIndex < 0) 
    {
        selectedIndex = selects.size() - 1;
    } 
    else if ((size_t) selectedIndex >= selects.size()) 
    {
        selectedIndex = 0;
    }

    // select new bone
    selects[selectedIndex]->setSelected(true);
}