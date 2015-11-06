#ifndef MODEL_H
#define MODEL_H

#include <iostream>
#include <vector>

#include <glm/glm.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "simple_ar_demo/shader.h"
#include "simple_ar_demo/Mesh.h"


class Model {
public:
    Model(std::string path) {
        this->loadModel(path);
    }

    void draw(Shader shader);

private:
    std::vector<Mesh> meshes;
    std::string directory;
    std::vector<Texture> textures_loaded;

    void loadModel(std::string path);

    void processNode(aiNode *node, const aiScene *scene);

    Mesh processMesh(aiMesh *mesh, const aiScene *scene);

    std::vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type, std::string typeName);
};

#endif