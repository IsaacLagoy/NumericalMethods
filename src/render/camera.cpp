#include "camera.h"

#include <cmath>

#define GLM_FORCE_RADIANS
#include <glm/gtc/matrix_transform.hpp>

Camera::Camera()
    : position(0.0f, 1.0f, 3.0f),
      yaw(0.0f),
      pitch(0.0f)
{
    lookAt(glm::vec3(0.0f, 0.0f, 0.0f));
}

glm::vec3 Camera::getForward() const
{
    const float cosPitch = std::cos(pitch);
    return glm::vec3(
        -std::sin(yaw) * cosPitch,
        -std::sin(pitch),
        -std::cos(yaw) * cosPitch
    );
}

void Camera::lookAt(const glm::vec3& target)
{
    const glm::vec3 direction = glm::normalize(target - position);
    yaw = std::atan2(-direction.x, -direction.z);
    pitch = std::asin(glm::clamp(-direction.y, -1.0f, 1.0f));
}

void Camera::processMouseMovement(float xOffset, float yOffset)
{
    yaw -= xOffset * mouseSensitivity;
    pitch -= yOffset * mouseSensitivity;
    pitch = glm::clamp(pitch, -maxPitch, maxPitch);
}

void Camera::processKeyboard(bool forward, bool backward, bool left, bool right, bool up, bool down, float deltaTime)
{
    const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
    glm::vec3 forwardFlat(-std::sin(yaw), 0.0f, -std::cos(yaw));
    forwardFlat = glm::normalize(forwardFlat);
    const glm::vec3 rightFlat = glm::normalize(glm::cross(forwardFlat, worldUp));

    glm::vec3 velocity(0.0f);
    if (forward) 
    {
        velocity += forwardFlat;
    }

    if (backward) 
    {
        velocity -= forwardFlat;
    }

    if (right) 
    {
        velocity += rightFlat;
    }

    if (left) 
    {
        velocity -= rightFlat;
    }

    if (up) 
    {
        velocity += worldUp;
    }

    if (down) 
    {
        velocity -= worldUp;
    }

    if (glm::length(velocity) > 0.0f) 
    {
        position += glm::normalize(velocity) * moveSpeed * deltaTime;
    }
}

glm::mat4 Camera::getViewMatrix() const
{
    const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
    const glm::vec3 forward = getForward();
    return glm::lookAt(position, position + forward, worldUp);
}
