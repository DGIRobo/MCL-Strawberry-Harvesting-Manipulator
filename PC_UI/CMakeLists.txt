cmake_minimum_required(VERSION 3.16)

project(PC_UI VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt 5/6 둘 다 대응 (필요 모듈: Widgets, [선택] PrintSupport)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets PrintSupport SerialPort)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets PrintSupport SerialPort)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        qcustomplot.cpp
        qcustomplot.h
        globalVariables.h
        globalVariables.cpp
        serialreceiver.h
        serialreceiver.cpp
)

# ---- 실행 파일 생성 (Qt6/Qt5/Android 분기 유지) ----
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(PC_UI
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET PC_UI APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(PC_UI SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(PC_UI
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_include_directories(PC_UI PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# ---- 링크 ----
target_link_libraries(PC_UI PRIVATE
  Qt${QT_VERSION_MAJOR}::Widgets
  Qt${QT_VERSION_MAJOR}::PrintSupport   # PDF/프린트 안 쓰면 지워도 됨
  Qt${QT_VERSION_MAJOR}::SerialPort
)

# ---- (중요) 빌드 오류 대책: big object 옵션 ----
# MinGW: 'too many sections' 회피
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND WIN32)
  set_source_files_properties(qcustomplot.cpp PROPERTIES
    COMPILE_FLAGS "-Wa,-mbig-obj -g0")
endif()
# MSVC: /bigobj
if (MSVC)
  target_compile_options(PC_UI PRIVATE /bigobj)
endif()

# ---- (선택) OpenGL 가속을 쓰고 싶다면 주석 해제 ----
# find_package(Qt${QT_VERSION_MAJOR} COMPONENTS OpenGLWidgets OPTIONAL_COMPONENTS)
# if (TARGET Qt${QT_VERSION_MAJOR}::OpenGLWidgets)
#   target_link_libraries(PC_UI PRIVATE Qt${QT_VERSION_MAJOR}::OpenGLWidgets)
#   target_compile_definitions(PC_UI PRIVATE QCUSTOMPLOT_USE_OPENGL)
# endif()

# ---- 번들/설치 설정 (원본 유지) ----
# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.PC_UI)
endif()
set_target_properties(PC_UI PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS PC_UI
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(PC_UI)
endif()
