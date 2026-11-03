set(srcs
    src/sdemu_engine.c
    src/sdemu_regs.c
    src/sdemu_crc.c
    src/sdemu_backend.c
    src/sdemu_fault.c)
set(incs include)
set(priv_requires "")

if(IDF_TARGET STREQUAL "esp32s3")
    list(APPEND srcs port/esp32s3/sdemu_esp.c)
    list(APPEND incs port/esp32s3/include)
    set(priv_requires esp_driver_gpio esp_driver_pcnt esp_hw_support hal freertos log)
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${incs}
                       PRIV_INCLUDE_DIRS src
                       PRIV_REQUIRES ${priv_requires}
                       LDFRAGMENTS linker.lf)

# Every cycle counts in the bit loop; -Og once cost 895 cycles at a byte boundary.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
