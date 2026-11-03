zephyr_library_named(esphome_api)

list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
include(nanopb)

# -x: api_options.proto only carries ESPHome's codegen options; nanopb ignores them.
set(NANOPB_OPTIONS "-x api_options.proto")
set(NANOPB_IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/proto)
zephyr_nanopb_sources(esphome_api proto/api.proto)

# Generated headers are included as <api.pb.h> by the server and its tests.
zephyr_include_directories(${CMAKE_CURRENT_BINARY_DIR}/proto)
