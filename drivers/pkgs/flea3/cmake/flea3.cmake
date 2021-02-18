
macro(FLEA3_ADD_EXE DIR NAME)
    add_executable(${PROJECT_NAME}_${NAME} src/${DIR}/${NAME}.cpp)
    target_link_libraries(${PROJECT_NAME}_${NAME} ${PROJECT_NAME})
#    set_target_properties(${NAME} PROPERTIES OUTPUT_NAME ${NAME} PREFIX ${PROJECT_NAME})
endmacro()
