#include "bone.h"
#include "../render/matrix_stack.h"
#include "../render/renderer.h"
#include "../render/obj_server.h"

#define GLM_FORCE_RADIANS
#include <glm/gtc/type_ptr.hpp>

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
    
    // draw bone
    MV.pushMatrix();                // push bone matrix

    MV.translate(bonePos);
    MV.rotate(boneRot);
    MV.scale(boneScale);

    // draw
    glBindBuffer(GL_ARRAY_BUFFER, Renderer::objServer.getMesh("cube").posBuf);
    glVertexAttribPointer(aPosLoc, 3, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glBindBuffer(GL_ARRAY_BUFFER, Renderer::objServer.getMesh("cube").norBuf);
    glVertexAttribPointer(aNorLoc, 3, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glUniformMatrix4fv(unifID, 1, GL_FALSE, glm::value_ptr(MV.topMatrix()));
    glDrawArrays(GL_TRIANGLES, 0, Renderer::objServer.getMesh("cube").indCount);

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
    boneRot = glm::quat(angle, axis) * boneRot;
}