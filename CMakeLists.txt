project(Path_Planning)

cmake_minimum_required(VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp)

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")

    include_directories("C:/Program Files/OpenSSL-Win64/include")
    include_directories("C:/Program Files/OpenSSL-Win64/lib")
    include_directories("C:/Program Files/xmrig-deps-4.1/gcc/x64/include")
    include_directories("C:/Program Files/xmrig-deps-4.1/gcc/x64/lib")
    include_directories("C:/Program Files/zlib-1.2.11")

    set(CMAKE_CXX_FLAGS, "${CXX_FLAGS} -fpermissive")

    add_definitions(-D_USE_MATH_DEFINES)
    add_definitions(-fpermissive)
endif (${CMAKE_SYSTEM_NAME} MATCHES "Windows")

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
