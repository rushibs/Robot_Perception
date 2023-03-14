# ----- Find dependencies -----

# filesystem
set(filesystem_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/filesystem/include)

# popl
set(popl_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/popl/include)

set(USE_STACK_TRACE_LOGGER OFF CACHE BOOL "Enable automatic stack trace logger of backward-cpp")

if(USE_STACK_TRACE_LOGGER)
    find_package(Backward REQUIRED)
    message(STATUS "Stack trace logger: ENABLED")
else()
    message(STATUS "Stack trace logger: DISABLED")
endif()

set(USE_GOOGLE_PERFTOOLS OFF CACHE BOOL "Enable profiler of google-perftools")

if(USE_GOOGLE_PERFTOOLS)
    find_package(Gperftools REQUIRED)
    include_directories(${GPERFTOOLS_INCLUDE_DIRS})
    message(STATUS "Google Perftools: ENABLED")
else()
    message(STATUS "Google Perftools: DISABLED")
endif()

# ----- Show dialog -----
if(USE_PANGOLIN_VIEWER)
    message(STATUS "Viewer for examples: PangolinViewer")
elseif(USE_SOCKET_PUBLISHER)
    message(STATUS "Viewer for examples: SocketPublisher")
else()
    message(STATUS "Viewer for examples: None")
endif()

# ----- Build example executables -----
set(EXECUTABLE_TARGETS "")

add_executable(cam_slam cam_slam.cc)
list(APPEND EXECUTABLE_TARGETS cam_slam)

add_executable(video_slam video_slam.cc)
list(APPEND EXECUTABLE_TARGETS video_slam)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    # Set output directory for executables
    set_target_properties(${EXECUTABLE_TARGET} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_BINARY_DIR}"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_BINARY_DIR}"
        RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PROJECT_BINARY_DIR}"
        RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_BINARY_DIR}")

    # PangolinViewer is used on a priority basis
    if(USE_PANGOLIN_VIEWER)
        # Set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE USE_PANGOLIN_VIEWER)

        # Link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE pangolin_viewer)
    elseif(USE_SOCKET_PUBLISHER)
        # Set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE USE_SOCKET_PUBLISHER)

        # Link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE socket_publisher)
    endif()

    # Setup stack trace logger
    if(USE_STACK_TRACE_LOGGER)
        target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE USE_STACK_TRACE_LOGGER)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE Backward::Backward)
    endif()

    # Setup google-perftools
    if(USE_GOOGLE_PERFTOOLS)
        target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE USE_GOOGLE_PERFTOOLS)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE ${GPERFTOOLS_LIBRARIES})
    endif()

    # Link stella_vslam
    target_link_libraries(${EXECUTABLE_TARGET}
        PRIVATE
        ${PROJECT_NAME}
        opencv_imgcodecs
        opencv_videoio)

    # include 3rd party library headers
    target_include_directories(${EXECUTABLE_TARGET}
        PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/popl/include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/filesystem/include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/spdlog/include>)
endforeach()
