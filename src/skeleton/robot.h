#ifndef ROBOT_H
#define ROBOT_H

#include "bone.h"

// controls bones and target navigation
// don't propagate scale in the matrix

constexpr float TARGET_SPEED = 1.0f;

class Robot {
private:
    Bone* root;

    std::vector<EndEffector*> endEffectors;
    int activeEndEffectorIndex = 0;

    // all bones that the solver is allowed to control
    std::vector<Bone*> actuatedJoints;

public:
    Robot();
    ~Robot();

    // rendering
    void draw(MatrixStack& MV, GLint aPosLoc, GLint aNorLoc, const GLint MVLoc);

    // end-effector / target control
    void iterate(int i);
    void processInput(bool forward, bool backward, bool left, bool right, bool up, bool down, float deltaTime);

    // solver joint control (local tangent deltas, right multiply)
    size_t getActuatedJointCount() const { return actuatedJoints.size(); }
    void applyJointDelta(size_t jointIndex, const glm::vec3& deltaR);
    void applyJointDeltas(const std::vector<glm::vec3>& deltaR);

    std::optional<glm::vec3> getWorldPosition(const Bone* bone) const;
    std::vector<glm::vec3> getEndEffectorPositions() const;
    std::vector<glm::vec3> getTargetPositions() const;
};

#endif // ROBOT_H