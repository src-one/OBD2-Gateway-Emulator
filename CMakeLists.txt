# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# (Not part of the boilerplate)
# This example uses an extra component for common functions such as Wi-Fi and Ethernet connection.
# set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)

set(EXTRA_COMPONENT_DIRS += $(PROJECT_PATH)/components)

set(MKFATFS_BIN mkfatfs)
set(FATFS_IMAGE_COMPONENT_PATH $(PROJECT_DIR)/components/fatfs_image)


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(can-demo)

#add_custom_command(
#    OUTPUT ${BUILD_DIR}/fatfs_image.list
#    WORKING_DIRECTORY $(FATFS_IMAGE_COMPONENT_PATH)
#    PRE_BUILD
#    COMMAND ${CMAKE_COMMAND} -E ls -la
#    DEPENDS $(FATFS_IMAGE_COMPONENT_PATH)/image
#    COMMENT "######### listing dir ..."
#)

# COMMAND ${CMAKE_COMMAND} -E $(PROJECT_DIR)/components/mkfatfs/src/$(MKFATFS_BIN) -c $(FATFS_IMAGE_COMPONENT_PATH)/image -s $(CONFIG_FATFS_SIZE) ${BUILD_DIR}/fatfs_image.img
    
#add_custom_target(mytest COMMAND make -C ${mytest_SOURCE_DIR}
#                         CLION_EXE_DIR=${PROJECT_BINARY_DIR})

# Test generation of a file to the build tree without full path.  As
# of CMake 2.6 custom command outputs specified by relative path go in
# the build tree.
add_custom_command(
  OUTPUT doc1.txt
  COMMAND ${CMAKE_COMMAND} -E echo "Example Document Target" > doc1.txt
  DEPENDS doc1.tex
  VERBATIM
  )
  
#add_custom_command(
#    WORKING_DIRECTORY $(PROJECT_PATH)
#    POST_BUILD
#    OUTPUT SVN_REVISION
#    COMMAND ${CMAKE_COMMAND} -E $(PROJECT_DIR)/components/mkfatfs/src/$(MKFATFS_BIN) -c $(FATFS_IMAGE_COMPONENT_PATH)/image -s $(CONFIG_FATFS_SIZE) ${BUILD_DIR}/fatfs_image.img
#
#    COMMENT "Making fatfs image ..."
#)

#add_custom_command(
#    COMMAND $(ESPTOOLPY_WRITE_FLASH) 0x$(CONFIG_FATFS_BASE_ADDR) $(BUILD_DIR_BASE)/fatfs_image.img
#    VERBATIM
#)
