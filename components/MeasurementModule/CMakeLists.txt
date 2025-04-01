
file(GLOB_RECURSE MeasurementModuleHeaders "include/*.h")
file(GLOB_RECURSE MeasurementModuleSource "*.cpp")
SET(MeasurementModule ${MeasurementModuleSource})

# Include the component's include directories
idf_component_register( SRCS ${MeasurementModule}
    INCLUDE_DIRS include
    REQUIRES arduino EventSystem
)
