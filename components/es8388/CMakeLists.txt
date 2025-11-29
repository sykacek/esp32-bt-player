set(component_srcs "es8388.c")

# get IDF version for comparison
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}")

idf_component_register(SRCS "${component_srcs}" REQUIRES esp_driver_i2s esp_driver_gpio esp_driver_i2c driver freertos INCLUDE_DIRS ".")
