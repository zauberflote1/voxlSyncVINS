

find_library(LIBMODAL_PIPE_SO NAMES libmodal_pipe.so HINTS /usr/lib /usr/lib64 )
find_library(LIBMODAL_JSON_SO NAMES libmodal_json.so HINTS /usr/lib /usr/lib64 )
find_library(LIBMODAL_JOURNAL_SO NAMES libmodal_journal.so HINTS /usr/lib /usr/lib64 )


set(TARGET camera-server-config-helper)
add_executable(${TARGET}
    camera-server-config-helper.cpp
    ../src/config_defaults.cpp
    ../src/config_file.cpp
)

target_link_libraries(camera-server-config-helper
    ${LIBMODAL_JOURNAL_SO}
    ${LIBMODAL_JSON_SO}
)
list(APPEND ALL_TARGETS ${TARGET})


# voxl-bayer-stats
set(TARGET voxl-bayer-stats)
add_executable(${TARGET} voxl-bayer-stats.c ../src/cv_routines.c ../src/misc.c)
target_link_libraries(${TARGET}
	pthread
	${LIBMODAL_PIPE_SO}
	${LIBMODAL_JSON_SO}
)
list(APPEND ALL_TARGETS ${TARGET})


# voxl-image-stats
set(TARGET voxl-image-stats)
add_executable(${TARGET} voxl-image-stats.c ../src/cv_routines.c ../src/misc.c)
target_link_libraries(${TARGET}
	m
	pthread
	${LIBMODAL_PIPE_SO}
	${LIBMODAL_JSON_SO}
)
list(APPEND ALL_TARGETS ${TARGET})


# install everything!
install(
	TARGETS ${ALL_TARGETS}
	LIBRARY			DESTINATION /usr/lib
	RUNTIME			DESTINATION /usr/bin
	PUBLIC_HEADER	DESTINATION /usr/include
)
