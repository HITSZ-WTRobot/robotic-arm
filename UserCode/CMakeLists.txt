cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM CPP and C languages
enable_language(CXX C ASM)

# 定义一个宏，用于递归收集所有头文件所在的目录并去重，结果存储在传入的变量中
MACRO(HEADER_DIRECTORIES return_list)
    FILE(GLOB_RECURSE new_list *.h)
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

# 递归收集User目录下自定义库的 include 路径
HEADER_DIRECTORIES(User_Include_Dirs)

list(LENGTH User_Include_Dirs header_dir_count)

message(STATUS "[INFO] 找到 ${header_dir_count} 个头文件目录")

# 使用GLOB_RECURSE递归收集User目录下所有源文件
file(GLOB_RECURSE User_Src 
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
)

# 使用message输出找到的源文件数量
list(LENGTH User_Src user_src_count)
message(STATUS "[INFO] 找到 ${user_src_count} 个源文件")

# 创建User接口库
add_library(user_modules INTERFACE)
# target_include_directories(user_modules INTERFACE ${User_Include_Dirs})
target_include_directories(user_modules INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)

# 将User源文件添加到主项目
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${User_Src})