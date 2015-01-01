#**********************
# detectMarker
#**********************

add_executable( detectMarker detectMarker.cpp )
target_link_libraries( detectMarker kinectAR ${ALVAR_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${OSG_LIBRARIES} )
set_target_properties( detectMarker PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

add_executable( detectMarker2 detectMarker2.cpp )
target_link_libraries( detectMarker2 kinectAR ${ALVAR_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${OSG_LIBRARIES} )
set_target_properties( detectMarker2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )


