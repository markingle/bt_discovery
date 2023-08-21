# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/markbingle/esp/esp-idf/components/bootloader/subproject"
  "/Users/markbingle/Downloads/IOTCode/Classic_BT_Demo/bt_discovery/build/bootloader"
  "/Users/markbingle/Downloads/IOTCode/Classic_BT_Demo/bt_discovery/build/bootloader-prefix"
  "/Users/markbingle/Downloads/IOTCode/Classic_BT_Demo/bt_discovery/build/bootloader-prefix/tmp"
  "/Users/markbingle/Downloads/IOTCode/Classic_BT_Demo/bt_discovery/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/markbingle/Downloads/IOTCode/Classic_BT_Demo/bt_discovery/build/bootloader-prefix/src"
  "/Users/markbingle/Downloads/IOTCode/Classic_BT_Demo/bt_discovery/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/markbingle/Downloads/IOTCode/Classic_BT_Demo/bt_discovery/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/markbingle/Downloads/IOTCode/Classic_BT_Demo/bt_discovery/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
