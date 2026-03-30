#define _USE_MATH_DEFINES

#include <COMRaDE/EntitySystem/EntitySystem.h>

#include <DataFrame/DataFrame.h>

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
Transform makeTranslation(double x, double y, double z)
{
    Transform transform = Transform::Identity();
    transform.setPosition(Eigen::Vector3d(x, y, z));
    return transform;
}

template<typename T>
T *requireEntityType(Entity *entity, const char *label)
{
    auto *typedEntity = dynamic_cast<T *>(entity);
    if (typedEntity == nullptr)
    {
        throw std::runtime_error(std::string("Failed to create entity: ") + label);
    }
    return typedEntity;
}
}

int main()
{
    try
    {
        Scene scene("minimal_bullet_scene");
        scene.setDescription("Minimal Bullet example: static floor cube, visual cylinder, dynamic sphere");

        SceneSimulationConfig config;
        config.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
        config.fixedTimeStep = 1.0 / 100.0;
        config.bulletInternalTimeStep = config.fixedTimeStep;
        config.maxSubSteps = 1;
        config.enableDynamics = true;
        scene.setSimulationConfig(config);

        EntityInfo floorInfo;
        floorInfo.type = EntityType::Cube;
        floorInfo.name = "floor";
        floorInfo.worldTransform = makeTranslation(0.0, 0.0, -0.5);
        floorInfo.a = 10.0;
        floorInfo.b = 10.0;
        floorInfo.c = 1.0;
        auto *floor = requireEntityType<Object>(scene.addEntity(floorInfo), "floor");
        floor->setMotionType(ObjectMotionType::Static);
        floor->setFriction(0.9);
        floor->setColor(Eigen::Vector4d(0.45, 0.45, 0.45, 1.0));

        EntityInfo cylinderInfo;
        cylinderInfo.type = EntityType::Cylinder;
        cylinderInfo.name = "visual_cylinder";
        cylinderInfo.worldTransform = makeTranslation(0.0, 0.0, 1.5);
        cylinderInfo.a = 0.3;
        cylinderInfo.b = 1.2;
        auto *visualCylinder = requireEntityType<Object>(scene.addEntity(cylinderInfo), "visual cylinder");
        visualCylinder->setMotionType(ObjectMotionType::VisualOnly);
        visualCylinder->setColor(Eigen::Vector4d(0.2, 0.6, 0.9, 1.0));

        EntityInfo sphereInfo;
        sphereInfo.type = EntityType::Sphere;
        sphereInfo.name = "dynamic_sphere";
        sphereInfo.worldTransform = makeTranslation(0.0, 0.0, 3.0);
        sphereInfo.a = 0.25;
        auto *dynamicSphere = requireEntityType<Object>(scene.addEntity(sphereInfo), "dynamic sphere");
        dynamicSphere->setMotionType(ObjectMotionType::Dynamic);
        dynamicSphere->setMass(10.0);
        dynamicSphere->setFriction(0.6);
        dynamicSphere->setRestitution(0.25);
        dynamicSphere->setDamping(0.02, 0.02);
        dynamicSphere->setColor(Eigen::Vector4d(0.9, 0.2, 0.2, 1.0));

        scene.initializeSimulation();

        std::vector<double> timeSamples;
        std::vector<double> heightSamples;
        timeSamples.reserve(126);
        heightSamples.reserve(126);

        const auto recordSphereHeight = [&](double timePoint) {
            timeSamples.push_back(timePoint);
            heightSamples.push_back(dynamicSphere->getWorldTransform().getPosition().z());
        };

        recordSphereHeight(0.0);
        constexpr double simulationDuration = 5.0;
        const double dt = config.fixedTimeStep;
        const std::size_t stepCount = static_cast<std::size_t>(simulationDuration / dt);
        for (std::size_t step = 0; step < stepCount; ++step)
        {
            scene.stepSimulation(dt);
            recordSphereHeight((step + 1) * dt);
        }

        namespace fs = std::filesystem;
        const fs::path outputDirectory = fs::current_path() / "example_output";
        fs::create_directories(outputDirectory);

        const fs::path sceneXmlPath = outputDirectory / "minimal_bullet_scene.xml";
        const fs::path heightCsvSourcePath = outputDirectory / "dynamic_sphere_height.csv";

        hmdf::StdDataFrame<double> heightFrame;
        heightFrame.load_index(std::move(timeSamples));
        heightFrame.load_column("height", std::move(heightSamples));
        heightFrame.write<double>(heightCsvSourcePath.string().c_str(), hmdf::io_format::csv2);

        SceneResourceEntry heightResource;
        heightResource.type = SceneResourceType::Result;
        heightResource.sourcePath = heightCsvSourcePath;
        heightResource.relativePath = fs::path("results") / heightCsvSourcePath.filename();
        heightResource.description = "Dynamic sphere height trajectory";
        scene.addResource(std::move(heightResource));

        if (!scene.saveToFile(sceneXmlPath))
        {
            std::cerr << "Failed to save scene to " << sceneXmlPath << std::endl;
            return 1;
        }

        std::cout << "Scene saved to: " << sceneXmlPath << std::endl;
        std::cout << "Height results saved as scene resource: "
                  << (sceneXmlPath.parent_path() / sceneXmlPath.stem() / "results" / heightCsvSourcePath.filename())
                  << std::endl;
        return 0;
    }
    catch (const std::exception &exception)
    {
        std::cerr << exception.what() << std::endl;
        return 1;
    }
}
