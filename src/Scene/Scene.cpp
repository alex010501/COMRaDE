#include <COMRaDE/Scene/Scene.h>
#include <COMRaDE/Scene/SceneLoader.h>
#include <COMRaDE/Scene/SceneSerializer.h>

#include <algorithm>
#include <charconv>
#include <sstream>
#include <system_error>
#include <utility>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <btBulletDynamicsCommon.h>
#include <tinyxml.h>

namespace
{
EntityType entityTypeFrom(const char* value)
{
    const std::string text = value != nullptr ? value : "";
    if (text == "Dummy") return EntityType::Dummy;
    if (text == "Folder") return EntityType::Folder;
    if (text == "Object") return EntityType::Object;
    if (text == "Cube") return EntityType::Cube;
    if (text == "Sphere") return EntityType::Sphere;
    if (text == "Cylinder") return EntityType::Cylinder;
    if (text == "MultiBody") return EntityType::MultiBody;
    if (text == "SceneRoot") return EntityType::SceneRoot;
    return EntityType::Entity;
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

ObjectMotionType motionTypeFrom(const char* value)
{
    const std::string text = value != nullptr ? value : "";
    if (text == "Static") return ObjectMotionType::Static;
    if (text == "Dynamic") return ObjectMotionType::Dynamic;
    return ObjectMotionType::VisualOnly;
}

ObjectShapeType shapeTypeFrom(const char* value)
{
    const std::string text = value != nullptr ? value : "";
    if (text == "Box") return ObjectShapeType::Box;
    if (text == "Sphere") return ObjectShapeType::Sphere;
    if (text == "Cylinder") return ObjectShapeType::Cylinder;
    return ObjectShapeType::Mesh;
}

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

Eigen::Vector3d parseVector3(const char* text)
{
    const std::vector<double> values = parseList(text);
    if (values.size() != 3)
    {
        return Eigen::Vector3d::Zero();
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
}

Eigen::Vector4d parseVector4(const char* text)
{
    const std::vector<double> values = parseList(text);
    if (values.size() != 4)
    {
        return Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);
    }
    return Eigen::Vector4d(values[0], values[1], values[2], values[3]);
}

bool parseDouble(const char* text, double& value)
{
    if (text == nullptr)
    {
        return false;
    }
    const std::string input(text);
    auto result = std::from_chars(input.data(), input.data() + input.size(), value);
    return result.ec == std::errc{} && result.ptr == input.data() + input.size();
}

bool parseU64(const char* text, std::uint64_t& value)
{
    if (text == nullptr)
    {
        return false;
    }
    const std::string input(text);
    auto result = std::from_chars(input.data(), input.data() + input.size(), value);
    return result.ec == std::errc{} && result.ptr == input.data() + input.size();
}

std::filesystem::path defaultRelativePath(SceneResourceType type, const std::filesystem::path& sourcePath)
{
    const auto name = sourcePath.filename();
    switch (type)
    {
    case SceneResourceType::Model: return std::filesystem::path("models") / name;
    case SceneResourceType::Script: return std::filesystem::path("scripts") / name;
    case SceneResourceType::Result: return std::filesystem::path("results") / name;
    case SceneResourceType::Generic:
    default: return std::filesystem::path("resources") / name;
    }
}

bool canImportModel(const std::filesystem::path& path)
{
    if (path.empty() || !std::filesystem::exists(path))
    {
        return false;
    }

    Assimp::Importer importer;
    return importer.ReadFile(path.string(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices) != nullptr;
}

bool sameResource(const SceneResourceEntry& lhs, const SceneResourceEntry& rhs)
{
    return lhs.type == rhs.type
        && lhs.sourcePath == rhs.sourcePath
        && lhs.relativePath == rhs.relativePath;
}

void collectEntitiesRecursive(Entity* entity, std::vector<Entity*>& out)
{
    if (entity == nullptr) return;
    out.push_back(entity);
    for (Entity* child : entity->getChildren())
    {
        collectEntitiesRecursive(child, out);
    }
}
}

Scene::Scene() : Scene("New scene") {}

Scene::Scene(std::string sceneName) :
    m_sceneName(std::move(sceneName)),
    m_sceneRoot((m_sceneName + " root").c_str())
{
    initializePhysicsWorld();
    m_selectedEntity = &m_sceneRoot;
}

Scene::~Scene()
{
    shutdownPhysicsWorld();
}

const std::string& Scene::getName() const noexcept { return m_sceneName; }
void Scene::setName(std::string sceneName) { m_sceneName = std::move(sceneName); }
const std::string& Scene::getDescription() const noexcept { return m_description; }
void Scene::setDescription(std::string description) { m_description = std::move(description); }
Entity* Scene::getSceneRoot() noexcept { return &m_sceneRoot; }
const Entity* Scene::getSceneRoot() const noexcept { return &m_sceneRoot; }
btDiscreteDynamicsWorld* Scene::getDynamicsWorld() noexcept { return m_dynamicsWorld; }
const btDiscreteDynamicsWorld* Scene::getDynamicsWorld() const noexcept { return m_dynamicsWorld; }
const SceneSimulationConfig& Scene::getSimulationConfig() const noexcept { return m_simulationConfig; }
void Scene::setSimulationConfig(const SceneSimulationConfig& config) { m_simulationConfig = config; applySceneConfigurationToPhysics(); }
void Scene::setGravity(const Eigen::Vector3d& gravity) { m_simulationConfig.gravity = gravity; applySceneConfigurationToPhysics(); }
const SceneSimulationState& Scene::getSimulationState() const noexcept { return m_simulationState; }
void Scene::resetSimulationState() { m_simulationState = SceneSimulationState{}; }
void Scene::initPhysics() { initializeSimulation(); }
void Scene::update(double dt) { stepSimulation(dt); }
void Scene::selectEntity(Entity* entity) { m_selectedEntity = entity != nullptr ? entity : &m_sceneRoot; }
Entity* Scene::getSelectedEntity() const noexcept { return m_selectedEntity; }
const std::vector<SceneResourceEntry>& Scene::getResources() const noexcept { return m_resources; }

void Scene::initializeSimulation()
{
    applySceneConfigurationToPhysics();
    m_sceneRoot.initPhysics(m_dynamicsWorld);
    m_simulationState.initialized = true;
    m_simulationState.phase = SimulationPhase::Idle;
}

void Scene::stepSimulation(double dt)
{
    if (!m_simulationState.initialized)
    {
        initializeSimulation();
    }
    m_simulationState.running = true;
    prepareSimulationStep(dt);
    runDynamicsStep(dt);
    synchronizeEntitiesAfterStep();
    ++m_simulationState.stepIndex;
    m_simulationState.simulatedTime += dt;
    m_simulationState.phase = SimulationPhase::Idle;
    m_simulationState.running = false;
}

void Scene::simulate(double duration)
{
    if (duration <= 0.0 || m_simulationConfig.fixedTimeStep <= 0.0)
    {
        return;
    }
    const auto steps = static_cast<std::uint64_t>(duration / m_simulationConfig.fixedTimeStep);
    for (std::uint64_t i = 0; i < steps; ++i)
    {
        stepSimulation(m_simulationConfig.fixedTimeStep);
    }
}

Entity* Scene::addEntity(const EntityInfo& info)
{
    EntityInfo resolved = info;
    Folder* parent = resolveParentFolder(info.parent != nullptr ? info.parent : m_selectedEntity);
    resolved.parent = parent;
    Entity* entity = makeEntity(resolved);
    if (entity != nullptr)
    {
        parent->addChild(entity);
        if (m_simulationState.initialized)
        {
            entity->initPhysics(m_dynamicsWorld);
            entity->update();
        }
    }
    return entity;
}

std::vector<Entity*> Scene::collectEntities() const
{
    std::vector<Entity*> entities;
    collectEntitiesRecursive(const_cast<SceneRoot*>(&m_sceneRoot), entities);
    return entities;
}

void Scene::addResource(SceneResourceEntry resource)
{
    if (resource.relativePath.empty() && !resource.sourcePath.empty())
    {
        resource.relativePath = defaultRelativePath(resource.type, resource.sourcePath);
    }
    m_resources.push_back(std::move(resource));
}

SceneStorageLayout Scene::makeStorageLayout(const std::filesystem::path& xmlFilePath) const
{
    SceneStorageLayout layout;
    layout.xmlFilePath = xmlFilePath;
    layout.resourceDirectory = xmlFilePath.parent_path() / xmlFilePath.stem();
    layout.modelsDirectory = layout.resourceDirectory / "models";
    layout.scriptsDirectory = layout.resourceDirectory / "scripts";
    layout.resultsDirectory = layout.resourceDirectory / "results";
    return layout;
}

bool Scene::saveToFile(const std::filesystem::path& xmlFilePath) const
{
    return SceneSerializer::save(*this, xmlFilePath);
}

bool Scene::loadFromFile(const std::filesystem::path& xmlFilePath)
{
    return SceneLoader::load(*this, xmlFilePath);
}

void Scene::initializePhysicsWorld()
{
    constexpr int maxProxies = 1024;
    m_broadphase = new btAxisSweep3(btVector3(-10000.0, -10000.0, -10000.0), btVector3(10000.0, 10000.0, 10000.0), maxProxies);
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    m_solver = new btSequentialImpulseConstraintSolver();
    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
    applySceneConfigurationToPhysics();
    m_groundBody = createGround();
    m_dynamicsWorld->addRigidBody(m_groundBody);
}

void Scene::shutdownPhysicsWorld()
{
    if (m_dynamicsWorld != nullptr && m_groundBody != nullptr) m_dynamicsWorld->removeRigidBody(m_groundBody);
    delete m_groundBody;
    delete m_groundMotionState;
    delete m_groundShape;
    delete m_dynamicsWorld;
    delete m_solver;
    delete m_dispatcher;
    delete m_collisionConfiguration;
    delete m_broadphase;
    m_groundBody = nullptr;
    m_groundMotionState = nullptr;
    m_groundShape = nullptr;
    m_dynamicsWorld = nullptr;
    m_solver = nullptr;
    m_dispatcher = nullptr;
    m_collisionConfiguration = nullptr;
    m_broadphase = nullptr;
}

btRigidBody* Scene::createGround()
{
    m_groundShape = new btStaticPlaneShape(btVector3(0.0, 0.0, 1.0), 0.0);
    m_groundMotionState = new btDefaultMotionState(Transform::Identity().toBtTransform());
    btRigidBody::btRigidBodyConstructionInfo info(0.0, m_groundMotionState, m_groundShape, btVector3(0.0, 0.0, 0.0));
    auto* body = new btRigidBody(info);
    body->setFriction(0.5);
    return body;
}

void Scene::clearSceneContent()
{
    m_sceneRoot.clearChildren();
    m_resources.clear();
    resetSimulationState();
}

void Scene::applySceneConfigurationToPhysics()
{
    if (m_dynamicsWorld != nullptr)
    {
        m_dynamicsWorld->setGravity(btVector3(m_simulationConfig.gravity.x(), m_simulationConfig.gravity.y(), m_simulationConfig.gravity.z()));
    }
}

void Scene::prepareSimulationStep(double dt)
{
    (void)dt;
    m_simulationState.phase = SimulationPhase::Prepare;

    for (Entity* entity : collectEntities())
    {
        auto* object = dynamic_cast<Object*>(entity);
        if (object != nullptr)
        {
            object->syncPhysicsTransform();
        }
    }
}

void Scene::runDynamicsStep(double dt)
{
    m_simulationState.phase = SimulationPhase::Dynamics;
    if (m_simulationConfig.enableDynamics && m_dynamicsWorld != nullptr)
    {
        m_dynamicsWorld->stepSimulation(dt, m_simulationConfig.maxSubSteps, m_simulationConfig.bulletInternalTimeStep);
    }
}

void Scene::synchronizeEntitiesAfterStep()
{
    m_simulationState.phase = SimulationPhase::Synchronization;
    m_sceneRoot.update();
}

Entity* Scene::makeEntity(const EntityInfo& info) const
{
    const char* name = info.name != nullptr ? info.name : "Entity";
    switch (info.type)
    {
    case EntityType::Dummy: return new Dummy(name, info.worldTransform, info.parent);
    case EntityType::Folder: return new Folder(name, info.worldTransform, info.parent);
    case EntityType::Object: return new Object(name, info.worldTransform, info.parent, info.modelPath);
    case EntityType::Cube: return new CubeObject(name, info.a, info.b, info.c, info.worldTransform, info.parent);
    case EntityType::Sphere: return new SphereObject(name, info.a, info.worldTransform, info.parent);
    case EntityType::Cylinder: return new CylinderObject(name, info.a, info.b, info.worldTransform, info.parent);
    case EntityType::MultiBody: return new MultiBody(name, info.worldTransform, info.parent);
    case EntityType::Entity:
    case EntityType::SceneRoot:
    default: return new Entity(name, info.worldTransform, info.parent);
    }
}

Entity* Scene::loadEntityFromXml(const TiXmlElement& element, const std::filesystem::path& resourceDirectory, Entity* parent)
{
    Transform localTransform = Transform::Identity();
    EntityInfo info;
    info.type = entityTypeFrom(element.Attribute("type"));
    info.name = element.Attribute("name") != nullptr ? element.Attribute("name") : "Entity";
    info.parent = parent;
    if (const TiXmlElement* local = element.FirstChildElement("LocalTransform"))
    {
        localTransform = Transform(parseMatrix(local->Attribute("matrix")));
    }
    info.worldTransform = parent != nullptr ? parent->getWorldTransform() * localTransform : localTransform;

    Entity* entity = makeEntity(info);
    if (entity != nullptr)
    {
        entity->setLocalTransform(localTransform);
    }
    auto* object = dynamic_cast<Object*>(entity);
    if (object != nullptr)
    {
        if (const TiXmlElement* data = element.FirstChildElement("ObjectData"))
        {
            object->setMotionType(motionTypeFrom(data->Attribute("motion")));
            if (const char* massText = data->Attribute("mass"))
            {
                double mass = 0.0;
                if (parseDouble(massText, mass)) object->setMass(mass);
            }
            object->setColor(parseVector4(data->Attribute("color")));
            const Eigen::Vector3d dims = parseVector3(data->Attribute("dimensions"));
            switch (shapeTypeFrom(data->Attribute("shape")))
            {
            case ObjectShapeType::Box: object->setBoxGeometry(dims.x(), dims.y(), dims.z()); break;
            case ObjectShapeType::Sphere: object->setSphereGeometry(dims.x()); break;
            case ObjectShapeType::Cylinder: object->setCylinderGeometry(dims.x(), dims.z()); break;
            case ObjectShapeType::Mesh: break;
            }

            if (const char* model = data->Attribute("modelPath"))
            {
                std::filesystem::path resolved = resourceDirectory / model;
                if (canImportModel(resolved))
                {
                    object->setModelPath(resolved.string().c_str());
                }
                else
                {
                    object->setModelPath(model);
                }
            }
        }
    }

    if (auto* folder = dynamic_cast<Folder*>(entity))
    {
        if (const TiXmlElement* children = element.FirstChildElement("Children"))
        {
            for (const TiXmlElement* child = children->FirstChildElement("Entity"); child != nullptr; child = child->NextSiblingElement("Entity"))
            {
                if (Entity* loaded = loadEntityFromXml(*child, resourceDirectory, folder))
                {
                    folder->addChild(loaded);
                }
            }
        }
    }
    return entity;
}

Folder* Scene::resolveParentFolder(Entity* preferredParent) const
{
    Entity* candidate = preferredParent != nullptr ? preferredParent : const_cast<SceneRoot*>(&m_sceneRoot);
    if (candidate != nullptr && (candidate->getEntityType() == EntityType::Folder || candidate->getEntityType() == EntityType::SceneRoot))
    {
        return static_cast<Folder*>(candidate);
    }
    return const_cast<SceneRoot*>(&m_sceneRoot);
}

std::vector<SceneResourceEntry> Scene::collectSerializableResources(const SceneStorageLayout& layout) const
{
    std::vector<SceneResourceEntry> resources = m_resources;
    for (Entity* entity : collectEntities())
    {
        const auto* object = dynamic_cast<const Object*>(entity);
        if (object == nullptr || object->getModelPath().empty()) continue;
        SceneResourceEntry resource;
        resource.type = SceneResourceType::Model;
        resource.sourcePath = object->getModelPath();
        resource.relativePath = defaultRelativePath(resource.type, resource.sourcePath);
        resource.description = object->getName();
        auto it = std::find_if(resources.begin(), resources.end(), [&](const SceneResourceEntry& existing) { return sameResource(existing, resource); });
        if (it == resources.end()) resources.push_back(std::move(resource));
    }

    for (auto& resource : resources)
    {
        if (resource.relativePath.empty() && !resource.sourcePath.empty()) resource.relativePath = defaultRelativePath(resource.type, resource.sourcePath);
        if (resource.relativePath.is_absolute()) resource.relativePath = std::filesystem::relative(resource.relativePath, layout.resourceDirectory);
    }
    return resources;
}



