
file(GLOB_RECURSE EventSystemHeaders "include/*.h" "include/*.hpp")
file(GLOB_RECURSE EventSystemSource "*.cpp")

SET(EventSystem ${EventSystemSource})

# Include the component's include directories
idf_component_register( SRCS ${EventSystem}
    INCLUDE_DIRS include
    REQUIRES esp_ringbuf
)
