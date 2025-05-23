zephyr_library_named(hal_alif_cmsis_dfp)

zephyr_include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/drivers/include
  ${CMAKE_CURRENT_SOURCE_DIR}/Device/drivers/include
  ${CMAKE_CURRENT_SOURCE_DIR}/Device/common/include
  ${CMAKE_CURRENT_SOURCE_DIR}/Device/core/M55_HP/include
  ${CMAKE_CURRENT_SOURCE_DIR}/Device/core/M55_HP/config
  ${CMAKE_CURRENT_SOURCE_DIR}/Device/E7/AE722F80F55D5XX
  ${CMAKE_CURRENT_SOURCE_DIR}/zephyr/include
)

zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/drivers/source/sd.c)
zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/drivers/source/sdio.c)
zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/drivers/source/sd_host.c)
zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/Device/common/source/system_utils.c)

