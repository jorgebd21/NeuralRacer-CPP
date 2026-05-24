cmake_minimum_required(VERSION 3.10)

# 1. Ponle un nombre al proyecto
project(NeuralRacer)

# 2. Configura el estándar de C++ (Pista: fíjate en el CXXFLAGS de tu Makefile)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 3. Dile a CMake dónde están tus cabeceras y dónde buscar las librerías locales
include_directories(include)
link_directories(lib)

# 4. Encuentra todos tus archivos fuente (.cpp)
file(GLOB SOURCES "src/*.cpp")

# 5. Crea el ejecutable llamado 'app' con los archivos fuente
add_executable(app ${SOURCES})

# 6. Vincula las librerías necesarias
# (Sustituye los ??? con GL, m, pthread, dl, rt, X11 tal como estaban en LDFLAGS)
target_link_libraries(app raylib GL m pthread dl rt X11)