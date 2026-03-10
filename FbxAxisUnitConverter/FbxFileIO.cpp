#include "FbxFileIO.h"
#include <iostream>

FbxScene* FbxFileIO::Import(FbxManager* manager, const std::string& path)
{
    std::string utf8Path = PathToUtf8(path);

    FbxScene* scene = FbxScene::Create(manager, "Scene");
    FbxImporter* importer = FbxImporter::Create(manager, "");
    if (!importer->Initialize(utf8Path.c_str(), -1, manager->GetIOSettings()))
    {
        std::string err = importer->GetStatus().GetErrorString();
        importer->Destroy();
        scene->Destroy();
        throw std::runtime_error(
            std::string("Failed to open input file: ") + path + "\n" + err);
    }
    importer->Import(scene);
    importer->Destroy();

    std::cout << "[Info] Loaded: " << path << "\n";
    return scene;
}

void FbxFileIO::Export(FbxManager* manager, FbxScene* scene, const std::string& path)
{
    std::string utf8Path = PathToUtf8(path);

    FbxExporter* exporter = FbxExporter::Create(manager, "");
    if (!exporter->Initialize(utf8Path.c_str(), -1, manager->GetIOSettings()))
    {
        exporter->Destroy();
        throw std::runtime_error(
            std::string("Failed to open output file: ") + path);
    }
    exporter->Export(scene);
    exporter->Destroy();

    std::cout << "[Info] Saved: " << path << "\n";
}

std::string FbxFileIO::PathToUtf8(const std::string& path)
{
#ifdef _WIN32
    char* utf8 = nullptr;
    FbxAnsiToUTF8(path.c_str(), utf8);
    if (!utf8) return path;
    std::string result(utf8);
    FbxFree(utf8);
    return result;
#else
    return path;
#endif
}
