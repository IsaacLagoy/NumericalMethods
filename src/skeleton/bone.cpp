#include "bone.h"
#include "../render/matrix_stack.h"
#include "../render/renderer.h"
#include "../render/obj_server.h"

#define GLM_FORCE_RADIANS
#include <glm/gtc/type_ptr.hpp>

#include <cmath>

glm::quat rotationVectorToQuat(const glm::vec3& r)
{
    const float angle = glm::length(r);
    if (angle < 1e-8f)
    {
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    }
    const glm::vec3 axis = r / angle;
    const float half = angle * 0.5f;
    const float s = std::sin(half);
    return glm::quat(std::cos(half), axis.x * s, axis.y * s, axis.z * s);
}

// ------------------------------------------------------------
// Bone class implementation
// ------------------------------------------------------------

Bone::Bone(const glm::vec3& jointPos, const glm::vec3& bonePos, const glm::quat& boneRot, const glm::vec3& boneScale)
    : jointPos(jointPos), bonePos(bonePos), boneRot(boneRot), boneScale(boneScale) {}

Bone::~Bone() 
{
    for (Bone* child : children) 
    {
        delete child;
    }
}

void Bone::draw(MatrixStack& MV, GLint aPosLoc, GLint aNorLoc, const GLuint unifID)
{
    // transform joint
    MV.pushMatrix();                // push joint matrix
    MV.translate(jointPos);

    // rotate outside to propagate down the hierarchy
    MV.rotate(boneRot);
    
    // draw bone
    MV.pushMatrix();                // push bone matrix

    MV.translate(bonePos);
    MV.scale(boneScale);

    // draw
    drawArrays(MV, aPosLoc, aNorLoc, unifID, "cube");

    MV.popMatrix();                // pop bone matrix
    
    // draw children
    for (Bone* child : children)
    {
        child->draw(MV, aPosLoc, aNorLoc, unifID);
    }

    MV.popMatrix();                // pop joint matrix
}

void Bone::addChild(Bone* child) 
{
    children.push_back(child);
}

void Bone::rotate(float angle, const glm::vec3& axis) 
{
    boneRot = glm::normalize(boneRot * glm::quat(angle, axis));
}

void Bone::drawArrays(MatrixStack& MV, GLint aPosLoc, GLint aNorLoc, const GLuint unifID, std::string meshName)
{
    glBindBuffer(GL_ARRAY_BUFFER, Renderer::objServer.getMesh(meshName).posBuf);
    glVertexAttribPointer(aPosLoc, 3, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glBindBuffer(GL_ARRAY_BUFFER, Renderer::objServer.getMesh(meshName).norBuf);
    glVertexAttribPointer(aNorLoc, 3, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glUniformMatrix4fv(unifID, 1, GL_FALSE, glm::value_ptr(MV.topMatrix()));
    glDrawArrays(GL_TRIANGLES, 0, Renderer::objServer.getMesh(meshName).indCount);
}

std::optional<glm::vec3> Bone::getWorldPosition(MatrixStack& MV, const Bone* target) const
{
    MV.pushMatrix();
    MV.translate(jointPos);
    MV.rotate(boneRot);

    if (this == target) 
    {
        const glm::vec3 localPoint = glm::vec3(0.0f);
        const glm::vec3 world = glm::vec3(MV.topMatrix() * glm::vec4(localPoint, 1.0f));
        MV.popMatrix();
        return world;
    }

    for (const Bone* child : children) 
    {
        if (auto pos = child->getWorldPosition(MV, target)) 
        {
            MV.popMatrix();
            return pos;
        }
    }

    MV.popMatrix();
    return std::nullopt;
}

// ------------------------------------------------------------
// EndEffector class implementation
// ------------------------------------------------------------

EndEffector::EndEffector(const glm::vec3& pos, const glm::vec3& scale) : Bone(
    pos, 
    glm::vec3(0.0f, 0.0f, 0.0f), 
    glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 
    scale
) {}

void EndEffector::rotate(float angle, const glm::vec3& axis)
{
    (void) angle;
    (void) axis;
}

void EndEffector::setRot(const glm::quat& rot)
{
    (void) rot;
}

void EndEffector::draw(MatrixStack& MV, GLint aPosLoc, GLint aNorLoc, const GLuint unifID)
{
    // transform joint (no local rotation — fixed on parent)
    MV.pushMatrix();
    MV.translate(jointPos);
    
    // draw bone
    MV.pushMatrix();                // push bone matrix

    MV.translate(bonePos);
    MV.scale(boneScale);

    if (active)
    {
        MV.scale(1.0f + 0.5 * A + 0.5 * A * glm::sin(F * glfwGetTime()));
    }

    // draw
    drawArrays(MV, aPosLoc, aNorLoc, unifID, "sphere");

    MV.popMatrix();                // pop bone matrix
    
    // draw children
    for (Bone* child : children)
    {
        child->draw(MV, aPosLoc, aNorLoc, unifID);
    }

    MV.popMatrix();                // pop joint matrix
}