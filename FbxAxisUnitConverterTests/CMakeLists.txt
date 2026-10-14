add_executable(FbxAxisUnitConverterTests
    TestMain.cpp
    TestHelpers.h
    Test_DeepHierarchy.cpp
    Test_IdentityConversion.cpp
    Test_FileIO.cpp
    Test_JapanesePath.cpp
    Test_NormalConversion.cpp
    Test_PreRotation.cpp
    Test_RotationOrder.cpp
    Test_SharedMesh.cpp
    Test_SkinnedMesh.cpp
    Test_UnitOnly.cpp
    Test_ZupToYup.cpp
)

target_link_libraries(FbxAxisUnitConverterTests PRIVATE
    fbxconverter_lib
    gtest
)

include(GoogleTest)
gtest_discover_tests(FbxAxisUnitConverterTests)
