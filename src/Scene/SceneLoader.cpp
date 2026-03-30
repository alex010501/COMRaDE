#include <COMRaDE/Scene/SceneLoader.h>

#include <COMRaDE/Scene/Scene.h>

#include <charconv>
#include <sstream>
#include <string>
#include <vector>
#include <tinyxml.h>

namespace
{
std::vector<double> parseList(const char* text)
{
    std::vector<double> values;
    if (text == nullptr)
    {
        return values;
    }

    std::istringstream stream(text);
    double value = 0.0;
    while (stream >> value)
    {
        values.push_back(value);
    }

    return values;
}

Eigen::Matrix4d parseMatrix(const char* text)
{
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    const std::vector<double> values = parseList(text);
    if (values.size() != 16)
    {
        return matrix;
    }

    int index = 0;
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            matrix(row, col) = values[index++];
        }
    }

    return matrix;
}

SceneResourceType resourceTypeFrom(const char* value)
{
    const std::string text = value != nullptr ? value : "";
    if (text == "Model")
    {
        return SceneResourceType::Model;
    }
    if (text == "Script")
    {
        return SceneResourceType::Script;
    }
    if (text == "Result")
    {
        return SceneResourceType::Result;
    }
    return SceneResourceType::Generic;
}

SimulationPhase phaseFrom(const char* value)
{
    const std::string text = value != nullptr ? value : "";
    if (text == "Prepare")
    {
        return SimulationPhase::Prepare;
    }
    if (text == "Dynamics")
    {
        return SimulationPhase::Dynamics;
    }
    if (text == "Synchronization")
    {
        return SimulationPhase::Synchronization;
    }
    return SimulationPhase::Idle;
}

bool parseDouble(const char* text, double& value)
{
    if (text == nullptr)
    {
        return false;
    }

    const std::string input(text);
    const auto result = std::from_chars(input.data(), input.data() + input.size(), value);
    return result.ec == std::errc{} && result.ptr == input.data() + input.size();
}

bool parseU64(const char* text, std::uint64_t& value)
{
    if (text == nullptr)
    {
        return false;
    }

    const std::string input(text);
    const auto result = std::from_chars(input.data(), input.data() + input.size(), value);
    return result.ec == std::errc{} && result.ptr == input.data() + input.size();
}

bool parseBool(const char* text, bool defaultValue = false)
{
    if (text == nullptr)
    {
        return defaultValue;
    }

    const std::string value(text);
    return value == "1" || value == "true" || value == "True";
}
}

bool SceneLoader::load(Scene& scene, const std::filesystem::path& xmlFilePath)
{
    if (xmlFilePath.empty())
    {
        return false;
    }

    TiXmlDocument document(xmlFilePath.string().c_str());
    if (!document.LoadFile())
    {
        return false;
    }

    const TiXmlElement* root = document.FirstChildElement("Scene");
    if (root == nullptr)
    {
        return false;
    }

    SceneStorageLayout layout = scene.makeStorageLayout(xmlFilePath);
    if (const char* resourceDirectory = root->Attribute("resourceDirectory"))
    {
        layout.resourceDirectory = xmlFilePath.parent_path() / resourceDirectory;
        layout.modelsDirectory = layout.resourceDirectory / "models";
        layout.scriptsDirectory = layout.resourceDirectory / "scripts";
        layout.resultsDirectory = layout.resourceDirectory / "results";
    }

    scene.clearSceneContent();

    scene.m_sceneName = root->Attribute("name") != nullptr ? root->Attribute("name") : "New scene";
    scene.m_description = root->Attribute("description") != nullptr ? root->Attribute("description") : "";
    scene.m_sceneRoot.rename(scene.m_sceneName + " root");
    scene.m_sceneRoot.setLocalTransform(Transform::Identity());

    scene.m_simulationConfig = SceneSimulationConfig{};
    if (const TiXmlElement* simulation = root->FirstChildElement("Simulation"))
    {
        if (const char* gravity = simulation->Attribute("gravity"))
        {
            const std::vector<double> values = parseList(gravity);
            if (values.size() == 3)
            {
                scene.m_simulationConfig.gravity = Eigen::Vector3d(values[0], values[1], values[2]);
            }
        }

        parseDouble(simulation->Attribute("fixedTimeStep"), scene.m_simulationConfig.fixedTimeStep);
        parseDouble(simulation->Attribute("bulletInternalTimeStep"), scene.m_simulationConfig.bulletInternalTimeStep);

        int maxSubSteps = scene.m_simulationConfig.maxSubSteps;
        simulation->QueryIntAttribute("maxSubSteps", &maxSubSteps);
        scene.m_simulationConfig.maxSubSteps = maxSubSteps;
        scene.m_simulationConfig.enableDynamics = parseBool(simulation->Attribute("enableDynamics"), scene.m_simulationConfig.enableDynamics);
    }

    scene.m_simulationState = SceneSimulationState{};
    if (const TiXmlElement* state = root->FirstChildElement("SimulationState"))
    {
        parseU64(state->Attribute("stepIndex"), scene.m_simulationState.stepIndex);
        parseDouble(state->Attribute("simulatedTime"), scene.m_simulationState.simulatedTime);
        scene.m_simulationState.phase = phaseFrom(state->Attribute("phase"));
        scene.m_simulationState.initialized = parseBool(state->Attribute("initialized"), scene.m_simulationState.initialized);
        scene.m_simulationState.running = parseBool(state->Attribute("running"), scene.m_simulationState.running);
    }

    scene.m_resources.clear();
    if (const TiXmlElement* resources = root->FirstChildElement("Resources"))
    {
        for (const TiXmlElement* resource = resources->FirstChildElement("Resource");
             resource != nullptr;
             resource = resource->NextSiblingElement("Resource"))
        {
            SceneResourceEntry entry;
            entry.type = resourceTypeFrom(resource->Attribute("type"));
            entry.relativePath = resource->Attribute("relativePath") != nullptr ? resource->Attribute("relativePath") : "";
            entry.description = resource->Attribute("description") != nullptr ? resource->Attribute("description") : "";
            if (!entry.relativePath.empty())
            {
                entry.sourcePath = layout.resourceDirectory / entry.relativePath;
            }
            scene.m_resources.push_back(std::move(entry));
        }
    }

    if (const TiXmlElement* hierarchy = root->FirstChildElement("Hierarchy"))
    {
        const TiXmlElement* sceneRoot = hierarchy->FirstChildElement("Entity");
        if (sceneRoot != nullptr && std::string(sceneRoot->Attribute("type") != nullptr ? sceneRoot->Attribute("type") : "") == "SceneRoot")
        {
            if (const char* rootName = sceneRoot->Attribute("name"))
            {
                scene.m_sceneRoot.rename(rootName);
            }

            if (const TiXmlElement* local = sceneRoot->FirstChildElement("LocalTransform"))
            {
                scene.m_sceneRoot.setLocalTransform(Transform(parseMatrix(local->Attribute("matrix"))));
            }

            if (const TiXmlElement* children = sceneRoot->FirstChildElement("Children"))
            {
                for (const TiXmlElement* child = children->FirstChildElement("Entity");
                     child != nullptr;
                     child = child->NextSiblingElement("Entity"))
                {
                    if (Entity* loaded = scene.loadEntityFromXml(*child, layout.resourceDirectory, &scene.m_sceneRoot))
                    {
                        scene.m_sceneRoot.addChild(loaded);
                    }
                }
            }
        }
        else
        {
            for (const TiXmlElement* child = hierarchy->FirstChildElement("Entity");
                 child != nullptr;
                 child = child->NextSiblingElement("Entity"))
            {
                if (Entity* loaded = scene.loadEntityFromXml(*child, layout.resourceDirectory, &scene.m_sceneRoot))
                {
                    scene.m_sceneRoot.addChild(loaded);
                }
            }
        }
    }

    scene.applySceneConfigurationToPhysics();
    scene.m_selectedEntity = &scene.m_sceneRoot;

    return true;
}

