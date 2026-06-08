#ifndef BONE_H
#define BONE_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glad/glad.h>
#include <vector>
#include <optional>

class MatrixStack;

class Bone {
protected:
    // transforms
    glm::vec3 jointPos;
    glm::vec3 bonePos; // relative
    glm::quat boneRot; // relative
    glm::vec3 boneScale; // absolute

    // hierarchy
    std::vector<Bone*> children;

public:
    Bone(const glm::vec3& jointPos, const glm::vec3& bonePos, const glm::quat& boneRot, const glm::vec3& boneScale);
    ~Bone();

    void addChild(Bone* child);

    void draw(MatrixStack& MV, GLint aPosLoc, GLint aNorLoc, const GLuint unifID);

    // rotation (only public control over the bone)
    void rotate(float angle, const glm::vec3& axis);
};

class Select : public Bone {
private:
    // goals
    bool selected = false;
    std::optional<glm::vec3> targetPos;

public:
    // bone and joint pos are the same, this object will be a sphere
    Select(const glm::vec3& pos, const glm::quat& rot, const glm::vec3& scale);

    bool isSelected() const { return selected; }
    void setSelected(bool selected) { this->selected = selected; }

    std::optional<glm::vec3> getTargetPos() const { return targetPos; }
    void setTargetPos(const std::optional<glm::vec3>& targetPos) { this->targetPos = targetPos; }
};

#endif // BONE_H