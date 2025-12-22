cmake_minimum_required(VERSION 3.16)

project(Camera-UI VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Quick Qml Gui SerialPort QuickControls2)

qt_standard_project_setup(REQUIRES 6.5)

qt_add_executable(appCamera-UI
    main.cpp
    arducamcontroller.cpp
    arducamcontroller.h
    frameProvider.h
)

qt_add_qml_module(appCamera-UI
    URI CameraUI
    VERSION 1.0
    QML_FILES
        Main.qml
)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
set_target_properties(appCamera-UI PROPERTIES
#    MACOSX_BUNDLE_GUI_IDENTIFIER com.example.appCamera-UI
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

target_link_libraries(appCamera-UI
    PRIVATE
        Qt6::Quick
        Qt6::SerialPort
        Qt6::QuickControls2
)

include(GNUInstallDirs)
install(TARGETS appCamera-UI
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
