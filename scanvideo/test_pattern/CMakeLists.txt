if (TARGET pico_scanvideo_dpi)
    add_executable(test_pattern
            test_pattern.c
            )

    target_compile_definitions(test_pattern PRIVATE
            #PSRAM_MUTEX=1
            #PSRAM_SPINLOCK=1
            PSRAM_ASYNC=1
            PSRAM_PIN_CS=16
            PSRAM_PIN_SCK=17
            PSRAM_PIN_MOSI=18
            PSRAM_PIN_MISO=19
            PICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS=336
    )
        
    target_link_libraries(test_pattern PRIVATE
            pico_multicore
            pico_stdlib
            pico_scanvideo_dpi
            rp2040-psram
    )
    
    pico_add_extra_outputs(test_pattern)

    pico_enable_stdio_uart(test_pattern 1)
    pico_enable_stdio_usb(test_pattern 1)

    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/rp2040-psram rp2040-psram)

endif ()
