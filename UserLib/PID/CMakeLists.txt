cmake_minimum_required(VERSION 3.22)

project(pid LANGUAGES C)

add_library(pid INTERFACE)

target_sources(pid INTERFACE)

target_include_directories(pid INTERFACE
        ./
)
