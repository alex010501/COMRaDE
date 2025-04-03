# set sources for example
set(SOURCES main.cpp)

add_executable(example ${SOURCES})

target_link_libraries(example PUBLIC ${PROJECT_NAME})