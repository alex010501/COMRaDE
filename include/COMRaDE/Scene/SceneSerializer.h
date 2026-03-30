#pragma once

#include <DLL_API.h>

#include <filesystem>

class Scene;

class DLL_API SceneSerializer
{
public:
    static bool save(const Scene& scene, const std::filesystem::path& xmlFilePath);
};
