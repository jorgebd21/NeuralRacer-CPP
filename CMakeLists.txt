cmake_minimum_required(VERSION 3.10)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "¡Error! No se permite compilar en la carpeta raíz. Por favor, crea una carpeta 'build' (mkdir build), entra en ella (cd build) y ejecuta 'cmake ..'")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1. Ponle un nombre al proyecto
project(NeuralRacer)

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
