set(COMPONENT_ADD_INCLUDEDIRS microcdr/include include)

file(GLOB_RECURSE COMPONENT_SRCS microcdr/src/c/*.c)

set(COMPONENT_REQUIRES lwip)

register_component()

