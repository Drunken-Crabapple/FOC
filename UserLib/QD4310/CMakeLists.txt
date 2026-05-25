cmake_minimum_required(VERSION 3.22)

project(qd4310 LANGUAGES C)

add_library(qd4310 INTERFACE)

target_sources(qd4310 INTERFACE QD4310.c)

target_include_directories(qd4310 INTERFACE
        ./
)
