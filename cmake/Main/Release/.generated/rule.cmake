# The following functions contains all the flags passed to the different build stages.

set(PACK_REPO_PATH "C:/Users/hjeon/.mchp_packs" CACHE PATH "Path to the root of a pack repository.")

function(Main_Release_Release_avr_gcc_assemble_rule target)
    set(options
        "-x"
        "assembler-with-cpp"
        "${MP_EXTRA_AS_PRE}"
        "-mmcu=atmega128"
        "-B${PACK_REPO_PATH}/Microchip/ATmega_DFP/3.4.282/gcc/dev/atmega128"
        "-c"
        "-Wa,--defsym=__MPLAB_BUILD=1${MP_EXTRA_AS_POST},--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--gdwarf-2")
    list(REMOVE_ITEM options "")
    target_compile_options(${target} PRIVATE "${options}")
    target_compile_definitions(${target}
        PRIVATE "DEBUG"
        PRIVATE "__ATmega128__"
        PRIVATE "Release=Release")
    target_include_directories(${target} PRIVATE "${PACK_REPO_PATH}/Microchip/ATmega_DFP/3.4.282/include")
endfunction()
function(Main_Release_Release_avr_gcc_assembleWithPreprocess_rule target)
    set(options
        "-x"
        "assembler-with-cpp"
        "${MP_EXTRA_AS_PRE}"
        "-mmcu=atmega128"
        "-B${PACK_REPO_PATH}/Microchip/ATmega_DFP/3.4.282/gcc/dev/atmega128"
        "-c"
        "-Wa,--defsym=__MPLAB_BUILD=1${MP_EXTRA_AS_POST},--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--gdwarf-2")
    list(REMOVE_ITEM options "")
    target_compile_options(${target} PRIVATE "${options}")
    target_compile_definitions(${target}
        PRIVATE "DEBUG"
        PRIVATE "__ATmega128__"
        PRIVATE "Release=Release")
    target_include_directories(${target} PRIVATE "${PACK_REPO_PATH}/Microchip/ATmega_DFP/3.4.282/include")
endfunction()
function(Main_Release_Release_avr_gcc_compile_rule target)
    set(options
        "-g"
        "-gdwarf-2"
        "${MP_EXTRA_CC_PRE}"
        "-mmcu=atmega128"
        "-B${PACK_REPO_PATH}/Microchip/ATmega_DFP/3.4.282/gcc/dev/atmega128"
        "-x"
        "c"
        "-c"
        "-funsigned-char"
        "-funsigned-bitfields"
        "-Os"
        "-ffunction-sections"
        "-fdata-sections"
        "-fpack-struct"
        "-fshort-enums"
        "-Wall")
    list(REMOVE_ITEM options "")
    target_compile_options(${target} PRIVATE "${options}")
    target_compile_definitions(${target}
        PRIVATE "DEBUG"
        PRIVATE "__ATmega128__"
        PRIVATE "NDEBUG"
        PRIVATE "Release=Release")
    target_include_directories(${target} PRIVATE "${PACK_REPO_PATH}/Microchip/ATmega_DFP/3.4.282/include")
endfunction()
function(Main_Release_Release_avr_gcc_compile_cpp_rule target)
    set(options
        "-g"
        "-gdwarf-2"
        "${MP_EXTRA_CC_PRE}"
        "-mmcu=atmega128"
        "-B${PACK_REPO_PATH}/Microchip/ATmega_DFP/3.4.282/gcc/dev/atmega128"
        "-x"
        "c++"
        "-c"
        "-O0"
        "-ffunction-sections"
        "-fdata-sections")
    list(REMOVE_ITEM options "")
    target_compile_options(${target} PRIVATE "${options}")
    target_compile_definitions(${target}
        PRIVATE "DEBUG"
        PRIVATE "__ATmega128__"
        PRIVATE "Release=Release")
    target_include_directories(${target} PRIVATE "${PACK_REPO_PATH}/Microchip/ATmega_DFP/3.4.282/include")
endfunction()
function(Main_Release_link_rule target)
    set(options
        "-gdwarf-2"
        "${MP_EXTRA_LD_PRE}"
        "-mmcu=atmega128"
        "-B${PACK_REPO_PATH}/Microchip/ATmega_DFP/3.4.282/gcc/dev/atmega128"
        "-o"
        "${FINAL_IMAGE_NAME}"
        "-Wl,--defsym=__MPLAB_BUILD=1${MP_EXTRA_LD_POST},--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1"
        "-Wl,--gc-sections")
    list(REMOVE_ITEM options "")
    target_link_options(${target} PRIVATE "${options}")
    target_compile_definitions(${target}
        PRIVATE "__ATmega128__"
        PRIVATE "Release=Release")
endfunction()
function(Main_Release_objcopy_ihex_rule target)
    add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND ${OBJCOPY}
        ARGS --output-target=ihex ${Main_Release_image_name} ${Main_Release_image_base_name}.hex
        WORKING_DIRECTORY ${Main_Release_output_dir})
endfunction()
function(Main_Release_objcopy_eep_rule target)
    add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND ${OBJCOPY}
        ARGS --only-section=.eeprom --change-section-lma .eeprom=0 --no-change-warnings --output-target=ihex ${Main_Release_image_name} ${Main_Release_image_base_name}.eep
        WORKING_DIRECTORY ${Main_Release_output_dir})
endfunction()
function(Main_Release_objcopy_lss_rule target)
    add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND ${OBJDUMP}
        ARGS --disassemble --wide --demangle --line-numbers --section-headers --source ${Main_Release_image_name} > ${Main_Release_image_base_name}.lss
        WORKING_DIRECTORY ${Main_Release_output_dir})
endfunction()
function(Main_Release_objcopy_srec_rule target)
    add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND ${OBJCOPY}
        ARGS --output-target=srec --remove-section=.eeprom --remove-section=.fuse --remove-section=.lock --remove-section=.signature ${Main_Release_image_name} ${Main_Release_image_base_name}.srec
        WORKING_DIRECTORY ${Main_Release_output_dir})
endfunction()
function(Main_Release_objcopy_sig_rule target)
    add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND ${OBJCOPY}
        ARGS --only-section=.user_signatures --change-section-lma .user_signatures=0 --no-change-warnings --output-target=ihex ${Main_Release_image_name} ${Main_Release_image_base_name}.usersignatures
        WORKING_DIRECTORY ${Main_Release_output_dir})
endfunction()
