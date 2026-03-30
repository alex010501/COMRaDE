# set sources for example
set(SOURCES main.cpp)

add_executable(example ${SOURCES})
comrade_set_output_dirs(example "${COMRADE_ROOT_BIN_DIR}/examples" "${COMRADE_ROOT_LIB_DIR}/examples")

target_link_libraries(example PUBLIC ${PROJECT_NAME})
