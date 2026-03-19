add_executable(unit_tests
    test_particle.cpp
    test_distance_constraint.cpp
    test_pin_constraint.cpp
    test_spatial_hash.cpp
    test_plane_collider.cpp
    test_sphere_collider.cpp
    test_config_loader.cpp
)

target_link_libraries(unit_tests 
    PRIVATE 
        ClothCore 
        GTest::gtest_main 
        Eigen3::Eigen
)

include(GoogleTest)
gtest_discover_tests(unit_tests)