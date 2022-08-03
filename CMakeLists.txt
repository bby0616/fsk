cmake_minimum_required(VERSION 3.16)
project(fsk C)

set(CMAKE_C_STANDARD 11)

add_library(fsk acom_spi_demod_v2.c )