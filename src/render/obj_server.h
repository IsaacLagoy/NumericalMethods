#ifndef OBJ_SERVER_H
#define OBJ_SERVER_H

// this class will manage objs

#include <string>
#include <unordered_map>
#include <glad/glad.h>

struct Mesh {    
    GLuint posBuf;
    GLuint norBuf;
    int indCount;
};

class ObjServer {
private:
    std::unordered_map<std::string, Mesh> meshMap;

public:
    ObjServer() = default;
    ~ObjServer() = default;

    void loadMesh(const std::string& meshName, const std::string& objPath);

    Mesh getMesh(const std::string& meshName);
};

#endif // OBJ_SERVER_H