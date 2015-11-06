#include "simple_ar_demo/Model.h"

#define STB_IMAGE_IMPLEMENTATION
#include <simple_ar_demo/stb_image.h>


GLint TextureFromFile(const char* path, std::string directory);


/**
 * Draw all the meshes
 */
void Model::draw(Shader shader) {
    for(GLuint i = 0; i < this->meshes.size(); ++i) {
        this->meshes[i].draw(shader);
    }
}


/**
 * Load model from file into a scene
 */
void Model::loadModel(std::string path) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);

    // Check the different flag error occurring during loading
    if(!scene || scene->mFlags == AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
    {
        std::cout << "ERROR::ASSIMP::" << importer.GetErrorString() << std::endl;
        return;
    }
    this->directory = path.substr(0, path.find_last_of('/'));

    this->processNode(scene->mRootNode, scene);
}


/**
 * Process node contained in a scene
 */
void Model::processNode(aiNode *node, const aiScene *scene) {
    for(GLuint i = 0; i < node->mNumMeshes; ++i) {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        this->meshes.push_back(this->processMesh(mesh, scene));
    }

    for(GLuint i = 0; i < node->mNumChildren; i++)
    {
        this->processNode(node->mChildren[i], scene);
    }
}


/**
 *  Create a Mesh object containing all Mesh info needed to be drawn
 */
Mesh Model::processMesh(aiMesh* mesh, const aiScene *scene) {
    std::vector<Vertex> vertices;
    std::vector<GLuint> indices;
    std::vector<Texture> textures;

    // Process vertices retrieve data: position, normals textures Coords
    for(GLuint i = 0; i < mesh->mNumVertices; ++i) {
        Vertex vertex;
        glm::vec3 vector;
        // Process vertex position
        vector.x = mesh->mVertices[i].x;
        vector.y = mesh->mVertices[i].y;
        vector.z = mesh->mVertices[i].z;
        vertex.Position = vector;

        // Process normals
        vector.x = mesh->mNormals[i].x;
        vector.y = mesh->mNormals[i].y;
        vector.z = mesh->mNormals[i].z;
        vertex.Normal = vector;

        // Process textureCoords - Only the first
        if(mesh->mTextureCoords[0]) {
            glm::vec2 texC;
            texC.x = mesh->mTextureCoords[0][i].x;
            texC.y = mesh->mTextureCoords[0][i].y;
            vertex.TexCoords = texC;
        }

        vertices.push_back(vertex);
    }
    // Process indices
    for(GLuint i = 0; i < mesh->mNumFaces; i++) {
        aiFace face = mesh->mFaces[i];

        for(GLuint j = 0; j < face.mNumIndices; j++)
            indices.push_back(face.mIndices[j]);
    }

    // Process Material
    if(mesh->mMaterialIndex >= 0) {
        aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];

        std::vector<Texture> diffuseMaps = this->loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
        textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());

        std::vector<Texture> specularMaps = this->loadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
        textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
    }

    return Mesh(vertices, indices, textures);
}


/**
 * Load all the textures contained in the scene
 */
std::vector<Texture> Model::loadMaterialTextures(aiMaterial *mat, aiTextureType type, std::string typeName) {
    std::vector<Texture> textures;

    for(GLuint i = 0; i < mat->GetTextureCount(type); i++) {
        aiString str;
        mat->GetTexture(type, i, &str);

        GLboolean skip = false;
        for(GLuint j = 0; j < textures_loaded.size(); j++)
        {
            if(textures_loaded[j].path == str)
            {
                textures.push_back(textures_loaded[j]);
                skip = true;
                break;
            }
        }

        if(!skip)
        {   // If texture hasn't been loaded already, load it
            Texture texture;
            texture.id = TextureFromFile(str.C_Str(), this->directory);
            texture.type = typeName;
            texture.path = str;
            textures.push_back(texture);
            std::cout << "loadMaterialTextures id:" << texture.id << " Type: " << texture.type << std::endl;
            this->textures_loaded.push_back(texture);  // Add to loaded textures
        }
    }

    return textures;
}


/**
 * Load a texture from a file
 */
GLint TextureFromFile(const char* path, std::string directory) {
    std::string filename = std::string(path);
    std::cout << "Loading texture " << filename;
    filename = directory + '/' + filename;
    GLuint textureID;
    glGenTextures(1, &textureID);
    int width,height, comp = 0;

    // FIXME stb_image seems to load unproperly
    static GLubyte checkImage[64][64][4];
    int i, j, c;

    for (i = 0; i < 64; i++) {
        for (j = 0; j < 64; j++) {
            c = ((((i&0x8)==0)^((j&0x8))==0))*255;
            checkImage[i][j][0] = (GLubyte) 255;
            checkImage[i][j][1] = (GLubyte) c;
            checkImage[i][j][2] = (GLubyte) c;
            checkImage[i][j][3] = (GLubyte) 255;
        }
    }


    unsigned char *img = 0;
    img = stbi_load(filename.c_str(), &width, &height, &comp, 0);

    std::cout << " (" << width << "x" << height << ") Comp: " << comp << std::endl;
    // Texture loading using stb_image
    glBindTexture(GL_TEXTURE_2D, textureID);

//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 64, 64, 0, GL_RGBA, GL_UNSIGNED_BYTE, checkImage);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img);

    glGenerateMipmap(GL_TEXTURE_2D);

    // Parameters
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    stbi_image_free(img);

    return textureID;
}
