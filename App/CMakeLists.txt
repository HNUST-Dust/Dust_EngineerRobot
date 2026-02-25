# Interaction

file(GLOB_RECURSE INTERACTION_SOURCES CONFIGURE_DEPENDS
    "*.c" "*.cpp" "*.s" "*.S"
)

# Legacy App modules below depend on removed drivers/headers (e.g. VT02.h, dvc_motor_dji.h)
# and are not part of the current bring-up path.
list(FILTER INTERACTION_SOURCES EXCLUDE REGEX ".*/commander\\.cpp$")
list(FILTER INTERACTION_SOURCES EXCLUDE REGEX ".*/booster\\.cpp$")

list(LENGTH INTERACTION_SOURCES INTERACTION_SOURCES_COUNT)
if(INTERACTION_SOURCES_COUNT EQUAL 0)
    add_library(dust_interaction INTERFACE)
    target_link_libraries(dust_interaction INTERFACE dust_options)
else()
    add_library(dust_interaction OBJECT)
    target_sources(dust_interaction PRIVATE ${INTERACTION_SOURCES})
    target_link_libraries(dust_interaction PRIVATE dust_options)
endif()
