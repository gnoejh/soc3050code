# cmake files support debug production
include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")

set(Main_Debug_library_list )

# Handle files with suffix s, for group Debug-avr-gcc
if(Main_Debug_Debug_avr_gcc_FILE_TYPE_assemble)
add_library(Main_Debug_Debug_avr_gcc_assemble OBJECT ${Main_Debug_Debug_avr_gcc_FILE_TYPE_assemble})
    Main_Debug_Debug_avr_gcc_assemble_rule(Main_Debug_Debug_avr_gcc_assemble)
    list(APPEND Main_Debug_library_list "$<TARGET_OBJECTS:Main_Debug_Debug_avr_gcc_assemble>")
endif()

# Handle files with suffix S, for group Debug-avr-gcc
if(Main_Debug_Debug_avr_gcc_FILE_TYPE_assembleWithPreprocess)
add_library(Main_Debug_Debug_avr_gcc_assembleWithPreprocess OBJECT ${Main_Debug_Debug_avr_gcc_FILE_TYPE_assembleWithPreprocess})
    Main_Debug_Debug_avr_gcc_assembleWithPreprocess_rule(Main_Debug_Debug_avr_gcc_assembleWithPreprocess)
    list(APPEND Main_Debug_library_list "$<TARGET_OBJECTS:Main_Debug_Debug_avr_gcc_assembleWithPreprocess>")
endif()

# Handle files with suffix [cC], for group Debug-avr-gcc
if(Main_Debug_Debug_avr_gcc_FILE_TYPE_compile)
add_library(Main_Debug_Debug_avr_gcc_compile OBJECT ${Main_Debug_Debug_avr_gcc_FILE_TYPE_compile})
    Main_Debug_Debug_avr_gcc_compile_rule(Main_Debug_Debug_avr_gcc_compile)
    list(APPEND Main_Debug_library_list "$<TARGET_OBJECTS:Main_Debug_Debug_avr_gcc_compile>")
endif()

# Handle files with suffix cpp, for group Debug-avr-gcc
if(Main_Debug_Debug_avr_gcc_FILE_TYPE_compile_cpp)
add_library(Main_Debug_Debug_avr_gcc_compile_cpp OBJECT ${Main_Debug_Debug_avr_gcc_FILE_TYPE_compile_cpp})
    Main_Debug_Debug_avr_gcc_compile_cpp_rule(Main_Debug_Debug_avr_gcc_compile_cpp)
    list(APPEND Main_Debug_library_list "$<TARGET_OBJECTS:Main_Debug_Debug_avr_gcc_compile_cpp>")
endif()

# Handle files with suffix elf, for group Debug-avr-gcc
if(Main_Debug_Debug_avr_gcc_FILE_TYPE_objcopy_ihex)
add_library(Main_Debug_Debug_avr_gcc_objcopy_ihex OBJECT ${Main_Debug_Debug_avr_gcc_FILE_TYPE_objcopy_ihex})
    Main_Debug_Debug_avr_gcc_objcopy_ihex_rule(Main_Debug_Debug_avr_gcc_objcopy_ihex)
    list(APPEND Main_Debug_library_list "$<TARGET_OBJECTS:Main_Debug_Debug_avr_gcc_objcopy_ihex>")
endif()

# Handle files with suffix elf, for group Debug-avr-gcc
if(Main_Debug_Debug_avr_gcc_FILE_TYPE_objcopy_eep)
add_library(Main_Debug_Debug_avr_gcc_objcopy_eep OBJECT ${Main_Debug_Debug_avr_gcc_FILE_TYPE_objcopy_eep})
    Main_Debug_Debug_avr_gcc_objcopy_eep_rule(Main_Debug_Debug_avr_gcc_objcopy_eep)
    list(APPEND Main_Debug_library_list "$<TARGET_OBJECTS:Main_Debug_Debug_avr_gcc_objcopy_eep>")
endif()

# Handle files with suffix elf, for group Debug-avr-gcc
if(Main_Debug_Debug_avr_gcc_FILE_TYPE_objcopy_lss)
add_library(Main_Debug_Debug_avr_gcc_objcopy_lss OBJECT ${Main_Debug_Debug_avr_gcc_FILE_TYPE_objcopy_lss})
    Main_Debug_Debug_avr_gcc_objcopy_lss_rule(Main_Debug_Debug_avr_gcc_objcopy_lss)
    list(APPEND Main_Debug_library_list "$<TARGET_OBJECTS:Main_Debug_Debug_avr_gcc_objcopy_lss>")
endif()

# Handle files with suffix elf, for group Debug-avr-gcc
if(Main_Debug_Debug_avr_gcc_FILE_TYPE_objcopy_srec)
add_library(Main_Debug_Debug_avr_gcc_objcopy_srec OBJECT ${Main_Debug_Debug_avr_gcc_FILE_TYPE_objcopy_srec})
    Main_Debug_Debug_avr_gcc_objcopy_srec_rule(Main_Debug_Debug_avr_gcc_objcopy_srec)
    list(APPEND Main_Debug_library_list "$<TARGET_OBJECTS:Main_Debug_Debug_avr_gcc_objcopy_srec>")
endif()

# Handle files with suffix elf, for group Debug-avr-gcc
if(Main_Debug_Debug_avr_gcc_FILE_TYPE_objcopy_sig)
add_library(Main_Debug_Debug_avr_gcc_objcopy_sig OBJECT ${Main_Debug_Debug_avr_gcc_FILE_TYPE_objcopy_sig})
    Main_Debug_Debug_avr_gcc_objcopy_sig_rule(Main_Debug_Debug_avr_gcc_objcopy_sig)
    list(APPEND Main_Debug_library_list "$<TARGET_OBJECTS:Main_Debug_Debug_avr_gcc_objcopy_sig>")
endif()

add_executable(Main_Debug_image_2NjCOKi3 ${Main_Debug_library_list})

set_target_properties(Main_Debug_image_2NjCOKi3 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Main_Debug_output_dir})
set_target_properties(Main_Debug_image_2NjCOKi3 PROPERTIES OUTPUT_NAME "Debug")
set_target_properties(Main_Debug_image_2NjCOKi3 PROPERTIES SUFFIX ".elf")

target_link_libraries(Main_Debug_image_2NjCOKi3 PRIVATE ${Main_Debug_Debug_avr_gcc_FILE_TYPE_link})

#Add objcopy steps
Main_Debug_objcopy_ihex_rule(Main_Debug_image_2NjCOKi3)
Main_Debug_objcopy_eep_rule(Main_Debug_image_2NjCOKi3)
Main_Debug_objcopy_lss_rule(Main_Debug_image_2NjCOKi3)
Main_Debug_objcopy_srec_rule(Main_Debug_image_2NjCOKi3)
Main_Debug_objcopy_sig_rule(Main_Debug_image_2NjCOKi3)

# Add the link options from the rule file.
Main_Debug_link_rule(Main_Debug_image_2NjCOKi3)



