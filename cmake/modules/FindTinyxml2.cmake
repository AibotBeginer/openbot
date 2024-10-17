# - Try to find TinyXML2
#
find_path (TinyXML2_DIR include/tinyxml2.h DOC "TinyXML2 Install Directory")

IF(EXISTS ${TinyXML2_DIR}/include/tinyxml2.h)
  SET(TinyXML2_FOUND YES)
  find_path(TinyXML2_INCLUDE_DIR tinyxml2.h PATH ${TinyXML2_DIR})
  find_library(TinyXML2_LIBRARIES NAMES tinyxml2 HINTS ${TinyXML2_DIR})
ELSE(EXISTS ${TinyXML2_DIR}/tinyxml2.h)
  SET(TinyXML2_FOUND NO)
ENDIF(EXISTS ${TinyXML2_DIR}/include/tinyxml2.h)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TinyXML2 DEFAULT_MSG TinyXML2_LIBRARIES TinyXML2_INCLUDE_DIR)
