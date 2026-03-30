#include <COMRaDE/Scene/SceneSerializer.h>

#include <COMRaDE/Scene/Scene.h>

#include <sstream>
#include <system_error>
#include <tinyxml.h>

namespace
{
std::string toString(EntityType type)
{
    switch (type)
    {
    case EntityType::Dummy: return "Dummy";
    case EntityType::Folder: return "Folder";
    case EntityType::Object: return "Object";
    case EntityType::Cube: return "Cube";
    case EntityType::Sphere: return "Sphere";
    case EntityType::Cylinder: return "Cylinder";
    case EntityType::MultiBody: return "MultiBody";
    case EntityType::SceneRoot: return "SceneRoot";
    case EntityType::Entity:
    default: return "Entity";
    }
}

std::string toString(ObjectMotionType type)
{
    switch (type)
    {
    case ObjectMotionType::Static: return "Static";
    case ObjectMotionType::Dynamic: return "Dynamic";
    case ObjectMotionType::VisualOnly:
    default: return "VisualOnly";
    }
}

std::string toString(ObjectShapeType type)
{
    switch (type)
    {
    case ObjectShapeType::Box: return "Box";
    case ObjectShapeType::Sphere: return "Sphere";
    case ObjectShapeType::Cylinder: return "Cylinder";
    case ObjectShapeType::Mesh:
    default: return "Mesh";
    }
}

std::string toString(SceneResourceType type)
{
    switch (type)
    {
    case SceneResourceType::Model: return "Model";
    case SceneResourceType::Script: return "Script";
    case SceneResourceType::Result: return "Result";
    case SceneResourceType::Generic:
    default: return "Generic";
    }
}

std::string toString(SimulationPhase phase)
{
    switch (phase)
    {
    case SimulationPhase::Prepare: return "Prepare";
    case SimulationPhase::Dynamics: return "Dynamics";
    case SimulationPhase::Synchronization: return "Synchronization";
    case SimulationPhase::Idle:
    default: return "Idle";
    }
}

std::string formatMatrix(const Eigen::Matrix4d& matrix)
{
    std::ostringstream stream;
    stream.precision(17);
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            if (row != 0 || col != 0)
            {
                stream << ' ';
            }
            stream << matrix(row, col);
        }
    }
    return stream.str();
}

std::string formatVector3(const Eigen::Vector3d& vector)
{
    std::ostringstream stream;
    stream.precision(17);
    stream << vector.x() << ' ' << vector.y() << ' ' << vector.z();
    return stream.str();
}

std::string formatVector4(const Eigen::Vector4d& vector)
{
    std::ostringstream stream;
    stream.precision(17);
    stream << vector.x() << ' ' << vector.y() << ' ' << vector.z() << ' ' << vector.w();
    return stream.str();
}

std::filesystem::path findModelRelativePath(
    const std::vector<SceneResourceEntry>& resources,
    const std::filesystem::path& modelPath)
{
    for (const SceneResourceEntry& resource : resources)
    {
        if (resource.type == SceneResourceType::Model && resource.sourcePath == modelPath && !resource.relativePath.empty())
        {
            return resource.relativePath;
        }
    }

    return {};
}

void appendEntityXml(TiXmlElement& parent, const Entity& entity, const std::vector<SceneResourceEntry>& resources)
{
    auto* node = new TiXmlElement("Entity");
    node->SetAttribute("type", toString(entity.getEntityType()));
    node->SetAttribute("name", entity.getName());

    auto* local = new TiXmlElement("LocalTransform");
    local->SetAttribute("matrix", formatMatrix(entity.getLocalTransform().matrix()));
    node->LinkEndChild(local);

    if (const auto* object = dynamic_cast<const Object*>(&entity))
    {
        auto* objectData = new TiXmlElement("ObjectData");
        objectData->SetAttribute("motion", toString(object->getMotionType()));
        objectData->SetAttribute("shape", toString(object->getShapeType()));
        objectData->SetDoubleAttribute("mass", object->getMass());
        objectData->SetAttribute("dimensions", formatVector3(object->getShapeDimensions()));
        objectData->SetAttribute("color", formatVector4(object->getColor()));

        if (!object->getModelPath().empty())
        {
            std::filesystem::path relativePath = findModelRelativePath(resources, object->getModelPath());
            if (relativePath.empty())
            {
                relativePath = std::filesystem::path("models") / std::filesystem::path(object->getModelPath()).filename();
            }
            objectData->SetAttribute("modelPath", relativePath.generic_string());
        }

        node->LinkEndChild(objectData);
    }

    const std::vector<Entity*> children = entity.getChildren();
    if (!children.empty())
    {
        auto* childList = new TiXmlElement("Children");
        for (const Entity* child : children)
        {
            if (child != nullptr)
            {
                appendEntityXml(*childList, *child, resources);
            }
        }
        node->LinkEndChild(childList);
    }

    parent.LinkEndChild(node);
}
}

bool SceneSerializer::save(const Scene& scene, const std::filesystem::path& xmlFilePath)
{
    if (xmlFilePath.empty())
    {
        return false;
    }

    const SceneStorageLayout layout = scene.makeStorageLayout(xmlFilePath);
    std::error_code errorCode;

    std::filesystem::create_directories(xmlFilePath.parent_path(), errorCode);
    if (errorCode)
    {
        return false;
    }

    std::filesystem::create_directories(layout.modelsDirectory, errorCode);
    if (errorCode)
    {
        return false;
    }

    std::filesystem::create_directories(layout.scriptsDirectory, errorCode);
    if (errorCode)
    {
        return false;
    }

    std::filesystem::create_directories(layout.resultsDirectory, errorCode);
    if (errorCode)
    {
        return false;
    }

    const std::vector<SceneResourceEntry> resources = scene.collectSerializableResources(layout);
    for (const SceneResourceEntry& resource : resources)
    {
        if (resource.sourcePath.empty() || !std::filesystem::exists(resource.sourcePath))
        {
            continue;
        }

        const std::filesystem::path destination = layout.resourceDirectory / resource.relativePath;
        std::filesystem::create_directories(destination.parent_path(), errorCode);
        if (errorCode)
        {
            return false;
        }

        const bool samePath =
            std::filesystem::weakly_canonical(resource.sourcePath, errorCode) ==
            std::filesystem::weakly_canonical(destination, errorCode);
        errorCode.clear();
        if (!samePath)
        {
            std::filesystem::copy_file(resource.sourcePath, destination, std::filesystem::copy_options::overwrite_existing, errorCode);
            if (errorCode)
            {
                return false;
            }
        }
    }

    TiXmlDocument document;
    document.LinkEndChild(new TiXmlDeclaration("1.0", "utf-8", ""));

    auto* root = new TiXmlElement("Scene");
    root->SetAttribute("name", scene.m_sceneName);
    root->SetAttribute("description", scene.m_description);
    root->SetAttribute("resourceDirectory", layout.resourceDirectory.filename().generic_string());
    document.LinkEndChild(root);

    auto* simulation = new TiXmlElement("Simulation");
    simulation->SetAttribute("gravity", formatVector3(scene.m_simulationConfig.gravity));
    simulation->SetDoubleAttribute("fixedTimeStep", scene.m_simulationConfig.fixedTimeStep);
    simulation->SetDoubleAttribute("bulletInternalTimeStep", scene.m_simulationConfig.bulletInternalTimeStep);
    simulation->SetAttribute("maxSubSteps", scene.m_simulationConfig.maxSubSteps);
    simulation->SetAttribute("enableDynamics", scene.m_simulationConfig.enableDynamics ? "true" : "false");
    root->LinkEndChild(simulation);

    auto* state = new TiXmlElement("SimulationState");
    state->SetAttribute("stepIndex", std::to_string(scene.m_simulationState.stepIndex));
    state->SetDoubleAttribute("simulatedTime", scene.m_simulationState.simulatedTime);
    state->SetAttribute("phase", toString(scene.m_simulationState.phase));
    state->SetAttribute("initialized", scene.m_simulationState.initialized ? "true" : "false");
    state->SetAttribute("running", scene.m_simulationState.running ? "true" : "false");
    root->LinkEndChild(state);

    auto* resourcesNode = new TiXmlElement("Resources");
    for (const SceneResourceEntry& resource : resources)
    {
        auto* resourceNode = new TiXmlElement("Resource");
        resourceNode->SetAttribute("type", toString(resource.type));
        resourceNode->SetAttribute("relativePath", resource.relativePath.generic_string());
        resourceNode->SetAttribute("description", resource.description);
        resourcesNode->LinkEndChild(resourceNode);
    }
    root->LinkEndChild(resourcesNode);

    auto* hierarchy = new TiXmlElement("Hierarchy");
    appendEntityXml(*hierarchy, scene.m_sceneRoot, resources);
    root->LinkEndChild(hierarchy);

    return document.SaveFile(xmlFilePath.string().c_str());
}


