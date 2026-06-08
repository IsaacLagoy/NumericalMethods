#include "obj_server.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>
#include <vector>

void ObjServer::loadMesh(const std::string& meshName, const std::string& objPath)
{
    // check if mesh already exists
    if (meshMap.find(meshName) != meshMap.end()) 
    {
        throw std::runtime_error("Mesh already exists");
    }

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warnStr, errStr;

    // load mesh
    std::string fileName = RESOURCE_DIR + objPath;
    std::vector<float> posBuf;
    std::vector<float> norBuf;

    int rc = tinyobj::LoadObj(&attrib, &shapes, &materials, &warnStr, &errStr, fileName.c_str());
    if (!rc) 
    {
        throw std::runtime_error("Failed to load mesh: " + errStr);
    }

    // process shapes
    for(size_t s = 0; s < shapes.size(); s++) 
    {
        size_t index_offset = 0;
        for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) 
        {
            size_t fv = shapes[s].mesh.num_face_vertices[f];
            for(size_t v = 0; v < fv; v++) 
            {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                posBuf.push_back(attrib.vertices[3*idx.vertex_index+0]);
                posBuf.push_back(attrib.vertices[3*idx.vertex_index+1]);
                posBuf.push_back(attrib.vertices[3*idx.vertex_index+2]);
                if(!attrib.normals.empty()) 
                {
                    norBuf.push_back(attrib.normals[3*idx.normal_index+0]);
                    norBuf.push_back(attrib.normals[3*idx.normal_index+1]);
                    norBuf.push_back(attrib.normals[3*idx.normal_index+2]);
                }
            }
            index_offset += fv;
        }
    }

    if (norBuf.size() != posBuf.size()) 
    {
        throw std::runtime_error("Normal buffer size does not match position buffer size");
    }

    int indCount = posBuf.size() / 3;
    GLuint posBufID, norBufID;
    glGenBuffers(1, &posBufID);
    glGenBuffers(1, &norBufID);

    glBindBuffer(GL_ARRAY_BUFFER, posBufID);
    glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, norBufID);
    glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    meshMap[meshName] = {posBufID, norBufID, indCount};
}

Mesh ObjServer::getMesh(const std::string& meshName)
{
    return meshMap[meshName];
}