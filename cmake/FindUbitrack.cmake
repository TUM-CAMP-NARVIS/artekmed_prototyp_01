# Locate Ubitrack - cmake install.
#
# This script defines:
#   UBITRACK_FOUND, set to 1 if found
#   UBITRACK_LIBRARY
#   UBITRACK_LIBRARIES
#   UBITRACK_INCLUDE_DIR
#   UBITRACK_INCLUDE_DIRS
#
# You can also use Ubitrack out of a source tree by specifying UBITRACK_SOURCE_DIR
# and UBITRACK_BUILD_DIR (in environment or CMake).


set( _libUbitrackSearchPaths
    /usr/local
    /usr
    /sw/ # Fink
    /opt/local # DarwinPorts
    /opt/csw # Blastwave
    /opt
    "$ENV{UBITRACK_PATH}/../"
    "C:/Libraries/Ubitrack"
    "C:/Program Files/Ubitrack"
    "C:/Program Files (x86)/Ubitrack"
    ~/Library/Frameworks
    /Library/Frameworks
)

macro( FIND_UBITRACK_HEADER MYHEADER MYHEADERNAME )
    mark_as_advanced( ${MYHEADER} )
    find_path( ${MYHEADER}
        ${MYHEADERNAME}
        HINTS
            ${UBITRACK_ROOT}
            $ENV{UBITRACK_ROOT}
            ${UBITRACK_SOURCE_DIR}
            $ENV{UBITRACK_SOURCE_DIR}
        PATH_SUFFIXES
	    src
            include
        PATHS
            ${_libUbitrackSearchPaths}
    )
    if( ${MYHEADER} )
        list( APPEND UBITRACK_INCLUDE_DIRS
            ${${MYHEADER}}
        )
    endif()
endmacro()

unset( UBITRACK_INCLUDE_DIR )
FIND_UBITRACK_HEADER( UBITRACK_INCLUDE_DIR utFacade/BasicFacade.h )



macro( FIND_UBITRACK_LIBRARY MYLIBRARY)
    #windows ubitrack adds version suffix e.g. 1.0.0 => 100
    FOREACH(MYLIBRARYNAME ${ARGN} )
		mark_as_advanced(${MYLIBRARY}_${MYLIBRARYNAME})
		mark_as_advanced(${MYLIBRARY}_${MYLIBRARYNAME}_d)
	    find_library( ${MYLIBRARY}_${MYLIBRARYNAME}
	        NAMES
	            ${MYLIBRARYNAME}
				${MYLIBRARYNAME}100 
	        HINTS
	            ${UBITRACK_ROOT}
	            $ENV{UBITRACK_ROOT}
	            ${UBITRACK_BUILD_DIR}
	            $ENV{UBITRACK_BUILD_DIR}
	        PATH_SUFFIXES
	            lib
	            lib32
	            lib64
	            lib/Release
	            bin
	            bin/Release
	        PATHS
	            ${_libUbitrackSearchPaths}
	    )
	    find_library( ${MYLIBRARY}_${MYLIBRARYNAME}_d
	        NAMES
	            ${MYLIBRARYNAME}d
	            ${MYLIBRARYNAME}100d
	        HINTS
	            ${UBITRACK_ROOT}
	            $ENV{UBITRACK_ROOT}
	            ${UBITRACK_BUILD_DIR}
	            $ENV{UBITRACK_BUILD_DIR}
	        PATH_SUFFIXES
	            lib
	            lib32
	            lib64
	            lib/Debug
	            bin
	            bin/Debug
	        PATHS
	            ${_libUbitrackSearchPaths}
	    )
	#    message( STATUS ${${MYLIBRARY}} ${${MYLIBRARY}_debug} )
	#    message( STATUS ${MYLIBRARYNAME} )

	    if( ${MYLIBRARY}_${MYLIBRARYNAME} )
	        list( APPEND UBITRACK_LIBRARIES
	            "optimized" ${${MYLIBRARY}_${MYLIBRARYNAME}}
	        )
	    endif()
	    if( ${MYLIBRARY}_${MYLIBRARYNAME}_d )
	        list( APPEND UBITRACK_LIBRARIES
	            "debug" ${${MYLIBRARY}_${MYLIBRARYNAME}_d}
	        )
	    endif()
	ENDFOREACH(MYLIBRARYNAME)
endmacro()

unset( UBITRACK_LIBRARIES )
FIND_UBITRACK_LIBRARY( UBITRACK_LIBRARY utfacade )
#FIND_UBITRACK_LIBRARY( UBITRACK_LIBRARY utcore utdataflow utvision utfacade )
#message( STATUS "Ubitrack Libraries: ${UBITRACK_LIBRARIES}" )

# handle the QUIETLY and REQUIRED arguments and set FMOD_FOUND to TRUE if all listed variables are TRUE
include( FindPackageHandleStandardArgs )
find_package_handle_standard_args(
    UBITRACK
    DEFAULT_MSG 
    UBITRACK_LIBRARIES
    UBITRACK_INCLUDE_DIR
	)
	
