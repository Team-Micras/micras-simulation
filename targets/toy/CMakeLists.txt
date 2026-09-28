###############################################################################
## Toy: the simulator's reference target, the smallest robot that uses every feature
###############################################################################

add_library(micras_sim_toy STATIC src/toy_target.cpp)
target_include_directories(micras_sim_toy PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(micras_sim_toy PUBLIC micras::sim_app)
target_compile_definitions(micras_sim_toy PRIVATE MICRAS_SIM_TOY_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(toy_sim src/main.cpp)
target_link_libraries(toy_sim PRIVATE micras_sim_toy)

micras_apply_warnings(micras_sim_toy toy_sim WERROR ${MICRAS_SIM_WERROR})

