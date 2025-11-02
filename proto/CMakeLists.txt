## Protobuf
set(protobuf_descriptors
  api.proto
  common.proto
  directions.proto
  info.proto
  options.proto
  sign.proto
  trip.proto
  transit.proto
  transit_fetch.proto
  incidents.proto
  status.proto
  matrix.proto
  isochrone.proto
  expansion.proto)

# Custom protobuf generation using protoc-gen-shimstructs plugin
# Based on the implementation in nob.c generate_protos()
function(protobuf_generate_cpp SRCS HDRS)
  set(proto_files ${ARGN})
  set(${SRCS})
  set(${HDRS})
  
  foreach(proto_file ${proto_files})
    # Get the stem (filename without .proto extension)
    get_filename_component(proto_stem ${proto_file} NAME_WE)
    get_filename_component(proto_dir ${proto_file} DIRECTORY)
    
    # Output paths match nob.c pattern: build/valhalla/{stem}.pb.cc and .pb.h
    set(output_cc "${CMAKE_CURRENT_BINARY_DIR}/${proto_stem}.pb.cc")
    set(output_h "${CMAKE_CURRENT_BINARY_DIR}/${proto_stem}.pb.h")

    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${proto_dir}")

    # Copy protobuf_shims.h to the output directory 
    configure_file(
      protobuf_shims.h
      ${CMAKE_CURRENT_BINARY_DIR}/protobuf_shims.h
      COPYONLY)
    
    list(APPEND ${SRCS} ${output_cc})
    list(APPEND ${HDRS} ${output_h})
    
    # Add custom command to generate the files
    add_custom_command(
      OUTPUT ${output_cc} ${output_h}
      COMMAND protoc
      ARGS -I ${CMAKE_CURRENT_SOURCE_DIR}
           --plugin=protoc-gen-shimstructs=${CMAKE_SOURCE_DIR}/proto/protoc-gen-shimstructs.py
           --shimstructs_out=${CMAKE_CURRENT_BINARY_DIR}
           ${CMAKE_CURRENT_SOURCE_DIR}/${proto_file}
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${proto_file}
      COMMENT "Generating protobuf sources for ${proto_file}"
      VERBATIM
    )
  endforeach()
  
  # Set parent scope variables
  set(${SRCS} ${${SRCS}} PARENT_SCOPE)
  set(${HDRS} ${${HDRS}} PARENT_SCOPE)
endfunction()

protobuf_generate_cpp(protobuf_srcs protobuf_hdrs ${protobuf_descriptors})

valhalla_module(NAME proto
  SOURCES
    ${protobuf_srcs}
  HEADERS
    ${protobuf_hdrs}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${CMAKE_CURRENT_BINARY_DIR}
    PRIVATE
      ${protozero_include_dir}
  DEPENDS
    ${valhalla_protobuf_targets})
