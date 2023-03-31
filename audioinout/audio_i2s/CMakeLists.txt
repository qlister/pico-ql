if (NOT TARGET audio_i2s_handler)
    message("In pico_audio_i2s")
    add_library(audio_i2s_handler INTERFACE)

    pico_generate_pio_header(audio_i2s_handler ${CMAKE_CURRENT_LIST_DIR}/audio_i2s.pio )

    target_sources(audio_i2s_handler INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/audio_i2s_handler.c
    )

    target_include_directories(audio_i2s_handler INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(audio_i2s_handler INTERFACE hardware_dma hardware_pio hardware_irq)

endif()