#ifndef BONE_H
#define BONE_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glad/glad.h>
#include <vector>
#include <optional>

constexpr float A = 0.1;
constexpr float F = 4.0;

class MatrixStack;

// Tangent-space rotation vector (axis * angle) to local quaternion delta.
glm::quat rotationVectorToQuat(const glm::vec3& r);

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
    virtual ~Bone();

    void addChild(Bone* child);

    virtual void draw(MatrixStack& MV, GLint aPosLoc, GLint aNorLoc, const GLuint unifID);

    // rotation (only public control over the bone; local space, right multiply)
    virtual void rotate(float angle, const glm::vec3& axis);

    // getters and setters
    glm::vec3 getPos() const { return jointPos; }
    glm::quat getRot() const { return boneRot; }
    glm::vec3 getScale() const { return boneScale; }

    void setPos(const glm::vec3& pos) { jointPos = pos; }
    virtual void setRot(const glm::quat& rot) { boneRot = rot; }
    void setScale(const glm::vec3& scale) { boneScale = scale; }

    // world position
    std::optional<glm::vec3> getWorldPosition(MatrixStack& MV, const Bone* bone) const;

    // draw convenience function
    static void drawArrays(MatrixStack& MV, GLint aPosLoc, GLint aNorLoc, const GLuint unifID, std::string meshName);
};

class EndEffector : public Bone {
private:
    bool active = false;
    glm::vec3 targetPos = glm::vec3(0.0f); // world space

public:
    // bone and joint pos are the same, this object will be a sphere
    EndEffector(const glm::vec3& pos, const glm::vec3& scale);

    void draw(MatrixStack& MV, GLint aPosLoc, GLint aNorLoc, const GLuint unifID) override;

    void rotate(float angle, const glm::vec3& axis) override;
    void setRot(const glm::quat& rot) override;

    bool isActive() const { return active; }
    void setActive(bool active) { this->active = active; }

    glm::vec3 getTargetPos() const { return targetPos; }
    void setTargetPos(const glm::vec3& targetPos) { this->targetPos = targetPos; }
};

#endif // BONE_H