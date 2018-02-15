cmake_minimum_required( VERSION 2.8 )
project( useSophus )

# 为使用 sophus，您需要使用find_pac令找到它kage命

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable( useSophus main.cpp )
target_link_libraries( useSophus ${Sophus_LIBRARIES} )