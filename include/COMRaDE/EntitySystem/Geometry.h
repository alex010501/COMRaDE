#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>

class Geometry {
public:
    enum class Kind {Primitive, Mesh, CAD };
    virtual ~Geometry() = default;

    virtual Kind getKind() const = 0;
};

// --- Примитив ---
class PrimitiveGeometry : public Geometry {
public:
    enum class Shape { Box, Sphere, Cylinder };

    PrimitiveGeometry(Shape shape, const Eigen::Vector3f& dims);

    Shape getShape() const;
    Eigen::Vector3f getDimensions() const;

    Kind getKind() const override { return Kind::Primitive; }

private:
    Shape shape;
    Eigen::Vector3f dimensions;
};

// --- Меш ---
class MeshGeometry : public Geometry {
public:
    struct SubMesh {
        std::vector<Eigen::Vector3f> vertices;
        std::vector<Eigen::Vector3f> normals;
        std::vector<Eigen::Vector2f> texCoords;
        std::vector<uint32_t> indices;
        std::string name;
    };

    MeshGeometry(const std::string& filePath);

    const std::string& getFilePath() const;
    const std::vector<Eigen::Vector3f>& getFlatVertices() const;
    const std::vector<uint32_t>& getFlatIndices() const;
    const std::vector<SubMesh>& getSubMeshes() const;

    Kind getKind() const override { return Kind::Mesh; }

private:
    std::string filePath;
    std::vector<Eigen::Vector3f> flatVertices;
    std::vector<uint32_t> flatIndices;
    std::vector<SubMesh> subMeshes;

    void loadFromFile(); // Assimp
};

// --- CAD ---
class CadGeometry : public Geometry {
public:
    CadGeometry(const std::string& filePath);

    const std::vector<Eigen::Vector3f>& getVertices() const;
    const std::vector<uint32_t>& getIndices() const;

    Kind getKind() const override { return Kind::CAD; }

private:
    std::string filePath;
    std::vector<Eigen::Vector3f> vertices;
    std::vector<uint32_t> indices;

    void loadStepViaOpenCascade(); // OCCT
};