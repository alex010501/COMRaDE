#pragma once

#include <DLL_API.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <COMRaDE/EntitySystem/Dummy.h>
#include <COMRaDE/EntitySystem/Folder.h>
#include <COMRaDE/EntitySystem/MultiBody.h>
#include <COMRaDE/EntitySystem/Object.h>

class btAxisSweep3;
class btCollisionShape;
class btCollisionDispatcher;
class btDefaultCollisionConfiguration;
class btDefaultMotionState;
class btDiscreteDynamicsWorld;
class btRigidBody;
class btSequentialImpulseConstraintSolver;

enum class SimulationPhase
{
    Idle,
    Prepare,
    Dynamics,
    Synchronization
};

struct SceneSimulationConfig
{
    Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
    double fixedTimeStep = 1.0e-3;
    double bulletInternalTimeStep = 1.0e-3;
    int maxSubSteps = 10;
    bool enableDynamics = true;
};

struct SceneSimulationState
{
    std::uint64_t stepIndex = 0;
    double simulatedTime = 0.0;
    SimulationPhase phase = SimulationPhase::Idle;
    bool initialized = false;
    bool running = false;
};

enum class SceneResourceType
{
    Model,
    Script,
    Result,
    Generic
};

struct SceneResourceEntry
{
    SceneResourceType type = SceneResourceType::Generic;
    std::filesystem::path sourcePath;
    std::filesystem::path relativePath;
    std::string description;
};

struct SceneStorageLayout
{
    std::filesystem::path xmlFilePath;
    std::filesystem::path resourceDirectory;
    std::filesystem::path modelsDirectory;
    std::filesystem::path scriptsDirectory;
    std::filesystem::path resultsDirectory;
};

class SceneSerializer;
class SceneLoader;

class DLL_API Scene
{
public:
    Scene();
    explicit Scene(std::string sceneName);
    ~Scene();

    const std::string& getName() const noexcept;
    void setName(std::string sceneName);

    const std::string& getDescription() const noexcept;
    void setDescription(std::string description);

    Entity* getSceneRoot() noexcept;
    const Entity* getSceneRoot() const noexcept;

    btDiscreteDynamicsWorld* getDynamicsWorld() noexcept;
    const btDiscreteDynamicsWorld* getDynamicsWorld() const noexcept;

    const SceneSimulationConfig& getSimulationConfig() const noexcept;
    void setSimulationConfig(const SceneSimulationConfig& config);
    void setGravity(const Eigen::Vector3d& gravity);

    const SceneSimulationState& getSimulationState() const noexcept;
    void resetSimulationState();
    void initializeSimulation();
    void stepSimulation(double dt);
    void simulate(double duration);
    void initPhysics();
    void update(double dt);

    Entity* addEntity(const EntityInfo& info);
    std::vector<Entity*> collectEntities() const;

    void selectEntity(Entity* entity);
    Entity* getSelectedEntity() const noexcept;

    void addResource(SceneResourceEntry resource);
    const std::vector<SceneResourceEntry>& getResources() const noexcept;

    SceneStorageLayout makeStorageLayout(const std::filesystem::path& xmlFilePath) const;
    bool saveToFile(const std::filesystem::path& xmlFilePath) const;
    bool loadFromFile(const std::filesystem::path& xmlFilePath);

private:
    void initializePhysicsWorld();
    void shutdownPhysicsWorld();
    btRigidBody* createGround();
    void clearSceneContent();

    void applySceneConfigurationToPhysics();
    void prepareSimulationStep(double dt);
    void runDynamicsStep(double dt);
    void synchronizeEntitiesAfterStep();

    Entity* makeEntity(const EntityInfo& info) const;
    Entity* loadEntityFromXml(const class TiXmlElement& element, const std::filesystem::path& resourceDirectory, Entity* parent);
    Folder* resolveParentFolder(Entity* preferredParent) const;

    std::vector<SceneResourceEntry> collectSerializableResources(const SceneStorageLayout& layout) const;

    friend class SceneSerializer;
    friend class SceneLoader;

    std::string m_sceneName = "New scene";
    std::string m_description;
    SceneSimulationConfig m_simulationConfig;
    SceneSimulationState m_simulationState;
    std::vector<SceneResourceEntry> m_resources;

    btDiscreteDynamicsWorld* m_dynamicsWorld = nullptr;
    btAxisSweep3* m_broadphase = nullptr;
    btDefaultCollisionConfiguration* m_collisionConfiguration = nullptr;
    btCollisionDispatcher* m_dispatcher = nullptr;
    btSequentialImpulseConstraintSolver* m_solver = nullptr;
    btRigidBody* m_groundBody = nullptr;
    btCollisionShape* m_groundShape = nullptr;
    btDefaultMotionState* m_groundMotionState = nullptr;

    SceneRoot m_sceneRoot;
    Entity* m_selectedEntity = nullptr;
};
