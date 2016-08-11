#ifndef MESH_H
#define MESH_H

#include <iostream>
#include <vector>

#include <glm/glm.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "basic_facade_demo/shader.h"

#include <assimp/Importer.hpp>

struct Vertex {
    glm::vec3 Position;
    glm::vec3 Normal;
    glm::vec2 TexCoords;
};


struct Texture {
    GLuint id;
    std::string type;
    aiString path;
};

/**
 * The structure can be easily modified thanks to fct offsetof(TYPE, MEMBER)
 * Vertex vertex;
 * vertex.Position = glm::vec3(0.2f, 0.4f, 0.6f);
 * vertex.Normal = glm::vec3(0.0f, 1.0f, 0.0f);
 * vertex.TexCoords = glm::vec2(1.0f, 0.0f);
 * // = [0.2f, 0.4f, 0.6f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f];
 */

class Mesh {
public:
    std::vector<Vertex> vertices;
    std::vector<GLuint> indices;
    std::vector<Texture> textures;

    Mesh(std::vector<Vertex> v, std::vector<GLuint> i, std::vector<Texture> t);
    void draw(Shader shader);

private:
    GLuint VAO, VBO, EBO;
    void setupMesh();
};

#endif