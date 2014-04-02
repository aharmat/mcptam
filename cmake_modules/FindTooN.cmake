# Find installed TooN

set(_TRY_FIND_TAG FALSE)
if (TooN_FIND_COMPONENTS)
   list(FIND TooN_FIND_COMPONENTS "tag" _find_tag)
   if (NOT (${_find_tag} EQUAL -1))
      set(_TRY_FIND_TAG TRUE)
   else()
      set(_TRY_FIND_TAG FALSE)
   endif()
endif()

if ((DEFINED TooN_FOUND) AND ((NOT _TRY_FIND_TAG) OR (DEFINED tag_FOUND)))
   if (TooN_FIND_REQUIRED AND (NOT TooN_FOUND))
      message (SEND_ERROR "Could not find TooN")
   endif()

   if (TooN_FIND_REQUIRED AND _TRY_FIND_TAG AND (NOT tag_FOUND))
      message (SEND_ERROR "Could not find tag")
   endif()

   return()
endif()

if (NOT (DEFINED TooN_FOUND))
   # find include path
   find_path(TooN_INCLUDE_DIR
      NAMES TooN/TooN.h
      DOC "TooN include path"
   )

   set(TooN_INCLUDE_DIRS ${TooN_INCLUDE_DIR})
   set(TooN_LIBRARIES)

   if (TooN_INCLUDE_DIR)
      set(TooN_FOUND TRUE)
      if (NOT TooN_FIND_QUIETLY)
         message (STATUS "Found TooN: ${TooN_INCLUDE_DIR}")
      endif()
   else()
      set(TooN_FOUND FALSE)
      if (TooN_FIND_REQUIRED)
         message(SEND_ERROR "Could not find TooN")
      endif()
   endif()
endif()

# --- If (COMPONENTS tag) was specified, then find tag too

if (_TRY_FIND_TAG AND (NOT (DEFINED tag_FOUND)))
   find_path(tag_INCLUDE_DIR
      NAMES tag/tuple.h
      DOC "tag include path"
   )
   find_library(tag_LIBRARY
      NAMES toontag
      DOC "tag library path"
   )

   find_package(clapack)

   #if (clapack_FOUND)

   if (TooN_FOUND AND tag_INCLUDE_DIR AND tag_LIBRARY AND clapack_FOUND)
      set(TooN_INCLUDE_DIRS ${TooN_INCLUDE_DIR} ${tag_INCLUDE_DIR} ${clapack_INCLUDE_DIRS})
      set(TooN_LIBRARIES ${tag_LIBRARY} ${clapack_LIBRARIES})
      set(tag_FOUND TRUE)
      if (NOT TooN_FIND_QUIETLY)
         message (STATUS "Found tag: ${tag_LIBRARY}")
      endif()
   else()
      set(tag_FOUND FALSE)
      if (TooN_FIND_REQUIRED)
         message (SEND_ERROR "Could not find tag")
      endif()
   endif()
endif()
