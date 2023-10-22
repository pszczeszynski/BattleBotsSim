#include "GuiUtils.h"


std::vector<GLuint> textures = {};

void ClearLastFrameTextures()
{
    for (GLuint texture : textures)
    {
        glDeleteTextures(1, &texture);
    }

    textures.clear();
}


/**
 * Converts a cv::Mat to an im gui texture.
 * @param mat the cv::Mat to convert
 * @return the ImTextureID of the converted texture
*/
ImTextureID MatToTexture(const cv::Mat& mat)
{
    // Convert the cv::Mat to RGBA format
    cv::Mat rgba;
    if (mat.channels() == 1)
    {
        cv::cvtColor(mat, rgba, cv::COLOR_GRAY2BGRA);
    }
    else
    {
        cv::cvtColor(mat, rgba, cv::COLOR_BGR2BGRA);
    }

    GLuint texture;
    glGenTextures(1, &texture);
    textures.push_back(texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rgba.cols, rgba.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, rgba.data);
    glBindTexture(GL_TEXTURE_2D, 0);

    return reinterpret_cast<ImTextureID>(static_cast<intptr_t>(texture));
}