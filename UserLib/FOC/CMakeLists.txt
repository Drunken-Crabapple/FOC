cmake_minimum_required(VERSION 3.22)

project(foc LANGUAGES C)

add_library(foc INTERFACE)

target_sources(foc INTERFACE FOC.c)

target_include_directories(foc INTERFACE
        ./
        ./base_classes
)
