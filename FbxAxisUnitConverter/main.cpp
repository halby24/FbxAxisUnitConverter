#include "ArgumentParser.h"
#include "FbxAxisUnitConverter.h"
#include "FbxFileIO.h"
#include "Logger.h"
#include <fbxsdk.h>
#include <iostream>

int main(int argc, char* argv[])
{
    ConsoleLogger logger;

    try
    {
        CliOptions cli = ArgumentParser::Parse(argc, argv);

        // FBX SDK のライフサイクルは CLI 側で管理する
        FbxManager* manager = FbxManager::Create();
        if (!manager) throw std::runtime_error("Failed to create FbxManager.");

        FbxIOSettings* ios = FbxIOSettings::Create(manager, IOSROOT);
        manager->SetIOSettings(ios);

        FbxScene* scene = FbxFileIO::Import(manager, cli.inputPath, &logger);

        FbxAxisUnitConverter converter(scene, cli.params, &logger);
        converter.Convert();

        FbxFileIO::Export(manager, scene, cli.outputPath, &logger);

        scene->Destroy();
        manager->Destroy();
        return 0;
    }
    catch (const std::exception& e)
    {
        logger.Error(e.what());
        return 1;
    }
}
