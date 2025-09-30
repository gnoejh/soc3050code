# The following variables contains the files used by the different stages of the build process.
set(Main_Release_Release_avr_gcc_FILE_TYPE_assemble)
set_source_files_properties(${Main_Release_Release_avr_gcc_FILE_TYPE_assemble} PROPERTIES LANGUAGE ASM)

# For assembly files, add "." to the include path for each file so that .include with a relative path works
foreach(source_file ${Main_Release_Release_avr_gcc_FILE_TYPE_assemble})
        set_source_files_properties(${source_file} PROPERTIES INCLUDE_DIRECTORIES "$<PATH:NORMAL_PATH,$<PATH:REMOVE_FILENAME,${source_file}>>")
endforeach()

set(Main_Release_Release_avr_gcc_FILE_TYPE_assembleWithPreprocess)
set_source_files_properties(${Main_Release_Release_avr_gcc_FILE_TYPE_assembleWithPreprocess} PROPERTIES LANGUAGE ASM)

# For assembly files, add "." to the include path for each file so that .include with a relative path works
foreach(source_file ${Main_Release_Release_avr_gcc_FILE_TYPE_assembleWithPreprocess})
        set_source_files_properties(${source_file} PROPERTIES INCLUDE_DIRECTORIES "$<PATH:NORMAL_PATH,$<PATH:REMOVE_FILENAME,${source_file}>>")
endforeach()

set(Main_Release_Release_avr_gcc_FILE_TYPE_compile
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/Main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/_adc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/_buzzer.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/_eeprom.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/_glcd.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/_init.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/_interrupt.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/_port.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/_timer2.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/_uart.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_accelerometer.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_adc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_blink.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_blink_asm.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_cds.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_game_hangman.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_game_obstacle.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_game_pong_uart_control.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_game_puzzle.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_game_word_puzzle.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_graphics.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_inline.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_interrupt.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_iot.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_joystick.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_memory.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_motors.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_port.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_serial.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_sound.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_sound_atari.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_sound_twingkle.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Main/main_timer.c")
set_source_files_properties(${Main_Release_Release_avr_gcc_FILE_TYPE_compile} PROPERTIES LANGUAGE C)
set(Main_Release_Release_avr_gcc_FILE_TYPE_compile_cpp)
set_source_files_properties(${Main_Release_Release_avr_gcc_FILE_TYPE_compile_cpp} PROPERTIES LANGUAGE CXX)
set(Main_Release_Release_avr_gcc_FILE_TYPE_link)
set(Main_Release_Release_avr_gcc_FILE_TYPE_objcopy_ihex)
set(Main_Release_Release_avr_gcc_FILE_TYPE_objcopy_eep)
set(Main_Release_Release_avr_gcc_FILE_TYPE_objcopy_lss)
set(Main_Release_Release_avr_gcc_FILE_TYPE_objcopy_srec)
set(Main_Release_Release_avr_gcc_FILE_TYPE_objcopy_sig)
set(Main_Release_image_name "Release.elf")
set(Main_Release_image_base_name "Release")

# The output directory of the final image.
set(Main_Release_output_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../out/Main")

# The full path to the final image.
set(Main_Release_full_path_to_image ${Main_Release_output_dir}/${Main_Release_image_name})
