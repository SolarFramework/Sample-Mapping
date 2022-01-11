## remove Qt dependencies
QT       -= core gui
CONFIG -= app_bundle qt

QMAKE_PROJECT_DEPTH = 0

## global defintions : target lib name, version
INSTALLSUBDIR = SolARBuild
TARGET = SolARPipelineMappingMultiNoDrop
FRAMEWORK = $${TARGET}
VERSION=0.11.0

DEFINES += MYVERSION=$${VERSION}
DEFINES += TEMPLATE_LIBRARY
CONFIG += c++1z

include(findremakenrules.pri)

include(../../manualincludepath.pri)

CONFIG(debug,debug|release) {
    DEFINES += _DEBUG=1
    DEFINES += DEBUG=1
}

CONFIG(release,debug|release) {
    DEFINES += _NDEBUG=1
    DEFINES += NDEBUG=1
}

DEPENDENCIESCONFIG = sharedlib
DEPENDENCIESCONFIG += install_recurse

## Configuration for Visual Studio to install binaries and dependencies. Work also for QT Creator by replacing QMAKE_INSTALL
PROJECTCONFIG = QTVS

#NOTE : CONFIG as staticlib or sharedlib, DEPENDENCIESCONFIG as staticlib or sharedlib and PROJECTDEPLOYDIR MUST BE DEFINED BEFORE templatelibbundle.pri inclusion
include ($$shell_quote($$shell_path($${QMAKE_REMAKEN_RULES_ROOT}/templatelibconfig.pri)))  # Shell_quote & shell_path required for visual on windows

## DEFINES FOR MSVC/INTEL C++ compilers
msvc {
DEFINES += "_BCOM_SHARED=__declspec(dllexport)"
}

INCLUDEPATH += interfaces/

SOURCES += \
    src/PipelineMappingMultiNoDropModule.cpp \
    src/PipelineMappingMultiNoDropProcessing.cpp

HEADERS += \
    interfaces/PipelineMappingMultiNoDropProcessing.h

unix:!android {
    QMAKE_CXXFLAGS += -Wignored-qualifiers
#    QMAKE_LINK=clang++
#    QMAKE_CXX = clang++
}

linux {
    QMAKE_LFLAGS += -ldl
}

macx {
    DEFINES += _MACOS_TARGET_
    QMAKE_MAC_SDK= macosx
    QMAKE_CFLAGS += -mmacosx-version-min=10.7 -std=c11 #-x objective-c++
    QMAKE_CXXFLAGS += -mmacosx-version-min=10.7 -std=c11 -std=c++11 -O3 -fPIC#-x objective-c++
    QMAKE_LFLAGS += -mmacosx-version-min=10.7 -v -lstdc++
    LIBS += -lstdc++ -lc -lpthread
}

win32 {

    DEFINES += WIN64 UNICODE _UNICODE
    QMAKE_COMPILER_DEFINES += _WIN64
    QMAKE_CXXFLAGS += -wd4250 -wd4251 -wd4244 -wd4275
    QMAKE_CXXFLAGS_DEBUG += /Od
    QMAKE_CXXFLAGS_RELEASE += /O2
}

android {
    ANDROID_ABIS="arm64-v8a"
}

header_files.path = $${PROJECTDEPLOYDIR}/interfaces
header_files.files = $$files($${PWD}/interfaces/*.h*)

xpcf_xml_files.path = $${USERHOMEFOLDER}/.xpcf/SolAR
xpcf_xml_files.files=$$files($${PWD}/*.xml)

INSTALLS += header_files
INSTALLS += xpcf_xml_files

OTHER_FILES += \
    packagedependencies.txt

DISTFILES += \
    LICENSE \
    README.md \
    bcom-SolARPipelineMappingNoDrop.pc.in \
    *.xml




#NOTE : Must be placed at the end of the .pro
include ($$shell_quote($$shell_path($${QMAKE_REMAKEN_RULES_ROOT}/remaken_install_target.pri)))) # Shell_quote & shell_path required for visual on windows


