cmake_minimum_required(VERSION 3.10)
project(main)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Ruta a donde descomprimiste SFML
set(SFML_DIR "C:/SFML-2.6.2/lib/cmake/SFML")  # Asegúrate de que aquí esté el archivo SFMLConfig.cmake

# Si usas versión estática de SFML, descomenta esto:
# set(SFML_STATIC_LIBRARIES TRUE)

# Buscar los paquetes SFML necesarios
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Tu ejecutable
add_executable(main main.cpp)

# Enlazar con las bibliotecas SFML
target_link_libraries(main sfml-graphics sfml-window sfml-system)

