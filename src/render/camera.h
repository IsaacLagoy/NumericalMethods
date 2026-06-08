#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>

class Camera {
private:
    glm::vec3 position;
    float yaw;
    float pitch;

    static constexpr float maxPitch = glm::radians(89.0f);
    static constexpr float moveSpeed = 3.0f;
    static constexpr float mouseSensitivity = 0.002f;

    glm::vec3 getForward() const;
    void lookAt(const glm::vec3& target);

public:
    Camera();
    ~Camera() = default;

    void processMouseMovement(float xOffset, float yOffset);
    void processKeyboard(bool forward, bool backward, bool left, bool right, bool up, bool down, float deltaTime);

    glm::mat4 getViewMatrix() const;
};

#endif
