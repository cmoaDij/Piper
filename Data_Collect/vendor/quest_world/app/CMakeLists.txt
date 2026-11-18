cmake_minimum_required(VERSION 3.22.1)
project(piperquestworld)
if(NOT META_SDK_SOURCE_DIR)
    include(FetchContent)
    FetchContent_Declare(meta_sdk
        GIT_REPOSITORY https://github.com/meta-quest/Meta-OpenXR-SDK.git
        GIT_TAG bbed2f20e38a5df7113630771c83cb8279e4fc26  # Meta SDK v85
        GIT_SHALLOW FALSE)
    FetchContent_MakeAvailable(meta_sdk)
    set(META_SDK_SOURCE_DIR ${meta_sdk_SOURCE_DIR})
endif()
add_subdirectory(${META_SDK_SOURCE_DIR}/Samples/3rdParty ${CMAKE_BINARY_DIR}/meta_3rdparty)
add_subdirectory(${META_SDK_SOURCE_DIR}/Samples/SampleXrFramework ${CMAKE_BINARY_DIR}/meta_framework)
# SDK v85 assumes Android onResume/onPause precede OpenXR READY/STOPPING.
# Both inverse orderings were observed on Quest 3 during sleep/wake and app
# foreground switching. OpenXR owns session transitions independently of the
# Android lifecycle. Remove only these two ordering assertions in a build-local
# copy, preserving xrBeginSession/xrEndSession and all SessionActive checks.
set(xr_app_source ${META_SDK_SOURCE_DIR}/Samples/SampleXrFramework/Src/XrApp.cpp)
file(READ ${xr_app_source} xr_app_code)
foreach(android_order_assert "        assert(Resumed);" "        assert(Resumed == false);")
    string(FIND "${xr_app_code}" "${android_order_assert}" assert_position)
    if(assert_position EQUAL -1)
        message(FATAL_ERROR "Meta SDK lifecycle assertion changed; review the local patch")
    endif()
    string(REPLACE "${android_order_assert}"
        "        // Android lifecycle notification may follow this OpenXR state change."
        xr_app_code "${xr_app_code}")
endforeach()
file(WRITE ${CMAKE_BINARY_DIR}/PiperXrApp.cpp "${xr_app_code}")
get_target_property(framework_sources samplexrframework SOURCES)
list(REMOVE_ITEM framework_sources ${xr_app_source})
list(APPEND framework_sources ${CMAKE_BINARY_DIR}/PiperXrApp.cpp)
set_property(TARGET samplexrframework PROPERTY SOURCES "${framework_sources}")
add_library(piperquestworld MODULE src/main.cpp)
target_link_libraries(piperquestworld PRIVATE samplexrframework android EGL GLESv3 log)
target_link_options(piperquestworld PRIVATE -u ANativeActivity_onCreate)
