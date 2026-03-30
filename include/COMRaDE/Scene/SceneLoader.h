#pragma once

#include <DLL_API.h>

#include <filesystem>

class Scene;

class DLL_API SceneLoader
{
public:
    static bool load(Scene& scene, const std::filesystem::path& xmlFilePath);
};
