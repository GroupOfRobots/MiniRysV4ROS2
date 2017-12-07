# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

# Add warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

execute_process (COMMAND uname -m COMMAND tr -d '\n\r' OUTPUT_VARIABLE ARCHITECTURE)
if ("${ARCHITECTURE}" STREQUAL "armhf" OR "${ARCHITECTURE}" STREQUAL "armv7l")
	message (STATUS "Detected architecture: armhf, applying optimization flags")
	add_compile_options(-mcpu=cortex-a8 -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8)
elseif ("${ARCHITECTURE}" STREQUAL "x86_64")
	message (STATUS "Detected architecture: amd64, no additional flags")
endif()
