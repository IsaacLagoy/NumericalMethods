#include "robot.h"
#include "../render/matrix_stack.h"

#include <algorithm>
#include <cfloat>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <GLFW/glfw3.h>

Robot::Robot() 
{
    // Torso at the origin, slightly taller than wide
    root = new Bone(
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        glm::vec3(1.0f, 1.5f, 1.0f)
    );

    // Head above torso
    Bone* head = new Bone(
        glm::vec3(0.0f, 0.5f, 0.0f),
        glm::vec3(0.0f, 0.5f, 0.0f),
        glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        glm::vec3(0.6f, 0.6f, 0.6f)
    );
    root->addChild(head);
    actuatedJoints.push_back(head);

    // Upper left arm attached to upper-left side of torso
    Bone* upperLeftArm = new Bone(
        glm::vec3(-0.5f, 0.4f, 0.0f),
        glm::vec3(-0.6f, 0.0f, 0.0f),
        glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        glm::vec3(1.2f, 0.3f, 0.3f)
    );
    root->addChild(upperLeftArm);
    actuatedJoints.push_back(upperLeftArm);

    // Lower left arm attached to end of upper left arm
    Bone* lowerLeftArm = new Bone(
        glm::vec3(-1.2f, 0.0f, 0.0f),
        glm::vec3(-0.4f, 0.0f, 0.0f),
        glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        glm::vec3(0.8f, 0.25f, 0.25f)
    );
    upperLeftArm->addChild(lowerLeftArm);
    actuatedJoints.push_back(lowerLeftArm);
    
    EndEffector* lowerLeftArmEndEffector = new EndEffector(
        glm::vec3(-0.8f, 0.0f, 0.0f),
        glm::vec3(0.5f, 0.5f, 0.5f)
    );
    lowerLeftArm->addChild(lowerLeftArmEndEffector);
    endEffectors.push_back(lowerLeftArmEndEffector);
    lowerLeftArmEndEffector->setActive(true);

    // Upper right arm (mirror of left)
    Bone* upperRightArm = new Bone(
        glm::vec3(0.5f, 0.4f, 0.0f), 
        glm::vec3(0.6f, 0.0f, 0.0f),
        glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        glm::vec3(1.2f, 0.3f, 0.3f)
    );
    root->addChild(upperRightArm);
    actuatedJoints.push_back(upperRightArm);

    // Lower right arm attached to end of upper right arm
    Bone* lowerRightArm = new Bone(
        glm::vec3(1.2f, 0.0f, 0.0f),
        glm::vec3(0.4f, 0.0f, 0.0f),
        glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        glm::vec3(0.8f, 0.25f, 0.25f)
    );
    upperRightArm->addChild(lowerRightArm);
    actuatedJoints.push_back(lowerRightArm);
    
    EndEffector* lowerRightArmEndEffector = new EndEffector(
        glm::vec3(0.8f, 0.0f, 0.0f),
        glm::vec3(0.5f, 0.5f, 0.5f)
    );
    lowerRightArm->addChild(lowerRightArmEndEffector);
    endEffectors.push_back(lowerRightArmEndEffector);

    // Upper left leg attached to bottom of torso
    Bone* upperLeftLeg = new Bone(
        glm::vec3(-0.25f, -0.4f, 0.0f), // left hip
        glm::vec3(0.0f, -0.9f, 0.0f),
        glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        glm::vec3(0.4f, 1.2f, 0.4f)
    );
    root->addChild(upperLeftLeg);
    actuatedJoints.push_back(upperLeftLeg);
    
    // Lower left leg attached to end of upper left leg
    Bone* lowerLeftLeg = new Bone(
        glm::vec3(0.0f, -1.5f, 0.0f),
        glm::vec3(0.0f, -0.5f, 0.0f),
        glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        glm::vec3(0.3f, 1.2f, 0.3f)
    );
    upperLeftLeg->addChild(lowerLeftLeg);
    actuatedJoints.push_back(lowerLeftLeg);

    EndEffector* lowerLeftLegEndEffector = new EndEffector(
        glm::vec3(0.0f, -1.2f, 0.0f),
        glm::vec3(0.5f, 0.5f, 0.5f)
    );
    lowerLeftLeg->addChild(lowerLeftLegEndEffector);
    endEffectors.push_back(lowerLeftLegEndEffector);

    // Upper right leg (mirror of left)
    Bone* upperRightLeg = new Bone(
        glm::vec3(0.25f, -0.4f, 0.0f),  // right hip
        glm::vec3(0.0f, -0.9f, 0.0f),
        glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        glm::vec3(0.4f, 1.2f, 0.4f)
    );
    root->addChild(upperRightLeg);
    actuatedJoints.push_back(upperRightLeg);
    
    // Lower right leg attached to end of upper right leg
    Bone* lowerRightLeg = new Bone(
        glm::vec3(0.0f, -1.5f, 0.0f),
        glm::vec3(0.0f, -0.5f, 0.0f),
        glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        glm::vec3(0.3f, 1.2f, 0.3f)
    );
    upperRightLeg->addChild(lowerRightLeg);
    actuatedJoints.push_back(lowerRightLeg);
    
    EndEffector* lowerRightLegEndEffector = new EndEffector(
        glm::vec3(0.0f, -1.2f, 0.0f),
        glm::vec3(0.5f, 0.5f, 0.5f)
    );
    lowerRightLeg->addChild(lowerRightLegEndEffector);
    endEffectors.push_back(lowerRightLegEndEffector);

    for (EndEffector* endEffector : endEffectors)
    {
        endEffector->setTargetPos(getWorldPosition(endEffector).value());
    }
}

Robot::~Robot() 
{
    delete root;
}

void Robot::draw(MatrixStack& MV, GLint aPosLoc, GLint aNorLoc, const GLint MVLoc)
{
    root->draw(MV, aPosLoc, aNorLoc, MVLoc);

    for (const EndEffector* endEffector : endEffectors)
    {
        glm::vec3 targetPos = endEffector->getTargetPos();

        MV.pushMatrix();

        MV.translate(targetPos);
        MV.scale(0.5f);

        if (endEffector->isActive())
        {
            MV.scale(1.0f + 0.5 * A + 0.5 * A * glm::sin(F * glfwGetTime()));
        }

        Bone::drawArrays(MV, aPosLoc, aNorLoc, MVLoc, "sphere");

        MV.popMatrix();
    }
}

void Robot::iterate(int i) {
    endEffectors[activeEndEffectorIndex]->setActive(false);

    activeEndEffectorIndex += i;
    if (activeEndEffectorIndex < 0) 
    {
        activeEndEffectorIndex = endEffectors.size() - 1;
    } 
    else if ((size_t) activeEndEffectorIndex >= endEffectors.size()) 
    {
        activeEndEffectorIndex = 0;
    }

    endEffectors[activeEndEffectorIndex]->setActive(true);
}

void Robot::processInput(bool forward, bool backward, bool left, bool right, bool up, bool down, float deltaTime)
{
    glm::vec3 targetVel(0.0f);
    if (forward)  targetVel.z += TARGET_SPEED;
    if (backward) targetVel.z -= TARGET_SPEED;
    if (left)     targetVel.x -= TARGET_SPEED;
    if (right)    targetVel.x += TARGET_SPEED;
    if (up)       targetVel.y += TARGET_SPEED;
    if (down)     targetVel.y -= TARGET_SPEED;
    targetVel *= deltaTime;

    if (glm::length(targetVel) < FLT_EPSILON)
    {
        return;
    }

    endEffectors[activeEndEffectorIndex]->setTargetPos(
        endEffectors[activeEndEffectorIndex]->getTargetPos() + targetVel
    );
}

// ------------------------------------------------------------
// Solver interface
// ------------------------------------------------------------

std::optional<glm::vec3> Robot::getWorldPosition(const Bone* bone) const
{
    MatrixStack MV;
    return root->getWorldPosition(MV, bone);
}

void Robot::applyJointDelta(size_t jointIndex, const glm::vec3& deltaR)
{
    if (jointIndex >= actuatedJoints.size())
    {
        return;
    }

    Bone* joint = actuatedJoints[jointIndex];
    const glm::quat delta = rotationVectorToQuat(deltaR);
    joint->setRot(glm::normalize(joint->getRot() * delta));
}

void Robot::applyJointDeltas(const std::vector<glm::vec3>& deltaR)
{
    const size_t n = std::min(deltaR.size(), actuatedJoints.size());
    for (size_t i = 0; i < n; ++i)
    {
        applyJointDelta(i, deltaR[i]);
    }
}

std::vector<glm::vec3> Robot::getTargetPositions() const
{
    std::vector<glm::vec3> targets;
    targets.reserve(endEffectors.size());
    for (const EndEffector* endEffector : endEffectors)
    {
        targets.push_back(endEffector->getTargetPos());
    }
    return targets;
}

std::vector<glm::vec3> Robot::getEndEffectorPositions() const
{
    std::vector<glm::vec3> positions;
    positions.reserve(endEffectors.size());
    for (const EndEffector* endEffector : endEffectors)
    {
        positions.push_back(getWorldPosition(endEffector).value());
    }
    return positions;
}
