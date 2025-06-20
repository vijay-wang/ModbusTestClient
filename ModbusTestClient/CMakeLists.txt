cmake_minimum_required(VERSION 3.5)

project(ModbusTestClient VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
find_package(Qt6 REQUIRED COMPONENTS SerialPort)

set(PROJECT_SOURCES
        main.cpp
        modbustest.cpp
        modbustest.h
        modelselectordialog.cpp
        modelselectordialog.h
        modbustest.ui
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(ModbusTestClient
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET ModbusTestClient APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(ModbusTestClient SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(ModbusTestClient
            ${PROJECT_SOURCES}
        )
    endif()
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ModbusTestClient PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(ModbusTestClient PRIVATE Qt6::SerialPort)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_library(MODBUS_LIBRARY modbus)

    if(MODBUS_LIBRARY)
        message(STATUS "Found libmodbus")
        target_link_libraries(ModbusTestClient PRIVATE modbus)
    else()
        message(FATAL_ERROR "libmodbus not found. Please run sudo apt install libmodbus-dev.")
    endif()

    if(Qt6SerialPort_FOUND)
        message(STATUS "Found serialport")
    else()
        message(FATAL_ERROR "serialport package not found. Please run sudo apt install libqt6serialport6-dev.")
    endif()
else()
    target_link_libraries(ModbusTestClient PRIVATE ${PROJECT_SOURCE_DIR}/lib/libmodbus-5.dll)
endif()


# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.ModbusTestClient)
endif()
set_target_properties(ModbusTestClient PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS ModbusTestClient
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(ModbusTestClient)
endif()

if(MSVC)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:CONSOLE")
elseif(MINGW)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mconsole")
endif()
