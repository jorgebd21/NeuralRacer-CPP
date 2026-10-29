cmake_minimum_required(VERSION 3.10)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "¡Error! No se permite compilar en la carpeta raíz. Por favor, crea una carpeta 'build' (mkdir build), entra en ella (cd build) y ejecuta 'cmake ..'")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1. Ponle un nombre al proyecto
project(NeuralRacer VERSION 2.4.0 LANGUAGES CXX)

# 2. Configura el estándar de C++ (Pista: fíjate en el CXXFLAGS de tu Makefile)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Enable AVX2 and FMA SIMD instruction sets
add_compile_options(-mavx2 -mfma)

# Definir dónde guardar los binarios (ejecutable en build/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_BUILD_RPATH "${CMAKE_SOURCE_DIR}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_SOURCE_DIR}/lib")


# 3. Dile a CMake dónde están tus cabeceras y dónde buscar las librerías locales
include_directories(include)
link_directories(lib)

# 4. Encuentra todos tus archivos fuente (.cpp)
file(GLOB SOURCES "src/*.cpp")

# 5. Crea el ejecutable llamado 'app' con los archivos fuente
add_executable(app ${SOURCES})

# 6. Vincula las librerías necesarias
find_package(OpenMP REQUIRED)
target_link_libraries(app raylib GL m pthread dl rt X11 tbb OpenMP::OpenMP_CXX)

# 7. Benchmark de alta frecuencia (0 asignaciones dinámicas por tick)
add_executable(bench_zero_alloc tests/bench_zero_alloc.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(bench_zero_alloc raylib GL m pthread dl rt X11)

# 8. Test de Dinámica Térmica de Neumáticos
add_executable(test_tire_thermals tests/test_tire_thermals.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(test_tire_thermals raylib GL m pthread dl rt X11)

# 9. Test de Acoplamiento Kamm-Térmico y Fricción
add_executable(test_kamm_thermal_coupling tests/test_kamm_thermal_coupling.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(test_kamm_thermal_coupling raylib GL m pthread dl rt X11)

# 10. Test de Exportación de Telemetría Térmica
add_executable(test_thermal_telemetry tests/test_thermal_telemetry.cpp src/Car.cpp src/TrackManager.cpp src/Telemetry.cpp)
target_link_libraries(test_thermal_telemetry raylib GL m pthread dl rt X11)

# 11. Test de Física de Trampas de Grava y Superficies
add_executable(test_gravel_physics tests/test_gravel_physics.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(test_gravel_physics raylib GL m pthread dl rt X11)

# Tests complementarios
add_executable(test_ringbuffer tests/test_ringbuffer.cpp)
add_executable(test_spatial_grid tests/test_spatial_grid.cpp)
add_executable(test_vehicle_spatial_grid tests/test_vehicle_spatial_grid.cpp)
add_executable(test_competitor_sensors tests/test_competitor_sensors.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(test_competitor_sensors raylib GL m pthread dl rt X11)
add_executable(test_drafting_aerodynamics tests/test_drafting_aerodynamics.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(test_drafting_aerodynamics raylib GL m pthread dl rt X11)
add_executable(test_interagent_collisions tests/test_interagent_collisions.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(test_interagent_collisions raylib GL m pthread dl rt X11)
add_executable(test_collision_perf tests/test_collision_perf.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(test_collision_perf raylib GL m pthread dl rt X11)
add_executable(test_mgu_cell tests/test_mgu_cell.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(test_mgu_cell raylib GL m pthread dl rt X11)
add_executable(test_speciation tests/test_speciation.cpp src/Evolution.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(test_speciation raylib GL m pthread dl rt X11)
add_executable(test_fitness_sharing tests/test_fitness_sharing.cpp src/Evolution.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(test_fitness_sharing raylib GL m pthread dl rt X11)
add_executable(test_genome_serialization tests/test_genome_serialization.cpp src/Evolution.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(test_genome_serialization raylib GL m pthread dl rt X11)
add_executable(test_speciation_diversity tests/test_speciation_diversity.cpp src/Evolution.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(test_speciation_diversity raylib GL m pthread dl rt X11)
add_executable(test_ghost_renderer tests/test_ghost_renderer.cpp src/Telemetry.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(test_ghost_renderer raylib GL m pthread dl rt X11)
add_executable(test_racing_line_heatmap tests/test_racing_line_heatmap.cpp src/Telemetry.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(test_racing_line_heatmap raylib GL m pthread dl rt X11)
add_executable(test_replay_scrubber tests/test_replay_scrubber.cpp)
target_link_libraries(test_replay_scrubber raylib GL m pthread dl rt X11)
add_executable(test_jacobian_visualizer tests/test_jacobian_visualizer.cpp src/Telemetry.cpp src/Car.cpp src/TrackManager.cpp)
target_link_libraries(test_jacobian_visualizer raylib GL m pthread dl rt X11)
add_executable(test_brain_simd tests/test_brain_simd.cpp)
add_executable(test_surface_dynamics tests/test_surface_dynamics.cpp)
add_executable(test_track_surfaces tests/test_track_surfaces.cpp src/TrackManager.cpp)
target_link_libraries(test_track_surfaces raylib GL m pthread dl rt X11)
