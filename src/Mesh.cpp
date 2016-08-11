#include "basic_facade_demo/Mesh.h"

Mesh::Mesh(std::vector<Vertex> v, std::vector<GLuint> i, std::vector<Texture> t) {
    this->vertices = v;
    this->indices = i;
    this->textures = t;

    this->setupMesh();
}

/**
 * Setup all the buffers needed, similar as regular setup
 */
void Mesh::setupMesh() {
//    std::cout << "===================" << std::endl;
//    std::cout << this->vertices.size() << std::endl;
//    std::cout << this->indices.size() << std::endl;
//    std::cout << this->textures.size() << std::endl;
//    std::cout << this->vertices[0].Position.x << std::endl;

    glGenVertexArrays(1, &this->VAO);
    glGenBuffers(1, &this->VBO);
    glGenBuffers(1, &this->EBO);

    glBindVertexArray(this->VAO);

    glBindBuffer(GL_ARRAY_BUFFER, this->VBO);
    glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(Vertex), &this->vertices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->indices.size() * sizeof(GLuint), &this->indices[0], GL_STATIC_DRAW);

    //Vertex Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*) 0);

    // Normal Position
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*) offsetof(Vertex, Normal));

    // TextCoord position
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*) offsetof(Vertex, TexCoords));

    // Be ready to be rendered !
    glBindVertexArray(0);
}


/**
 * Draw the meshes using some lightning
 *
 * We cant know how many textures we'll bind (with N from 1 to N):
 * uniform sampler2D texture_diffuseN
 * texture sampler2D texture_specularN
 *
 */
void Mesh::draw(Shader shader) {
    // Bind appropriate textures
    GLuint diffuseNr = 1;
    GLuint specularNr = 1;
    for(GLuint i = 0; i < this->textures.size(); i++) {
        glActiveTexture(GL_TEXTURE0 + i); // Active proper texture unit before binding
        // Retrieve texture number (the N in diffuse_textureN)
        std::stringstream ss;
        std::string number;
        std::string name = this->textures[i].type;
        if(name == "texture_diffuse")
            ss << diffuseNr++; // Transfer GLuint to stream
        else if(name == "texture_specular")
            ss << specularNr++; // Transfer GLuint to stream
        number = ss.str();
        // Now set the sampler to the correct texture unit
        glUniform1f(glGetUniformLocation(shader.Program, (name + number).c_str()), i);
        // And finally bind the texture
        glBindTexture(GL_TEXTURE_2D, this->textures[i].id);
    }

    // Draw mesh
    glBindVertexArray(this->VAO);
    glDrawElements(GL_TRIANGLES, this->indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    // Always good practice to set everything back to defaults once configured.
    for(GLuint i = 0; i < this->textures.size(); i++) {
        glActiveTexture(GL_TEXTURE0 + i);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
}
