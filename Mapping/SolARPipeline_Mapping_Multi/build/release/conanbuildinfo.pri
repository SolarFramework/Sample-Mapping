CONAN_INCLUDEPATH += "C:/.conan/d97b85/1/include" \
    "C:/.conan/d97b85/1/include/eigen3" \
    "C:/.conan/bdd45f/1/include"
CONAN_LIBS += -lboost_wave -lboost_container -lboost_contract -llibboost_exception -lboost_graph -lboost_iostreams -lboost_locale -lboost_log -lboost_program_options -lboost_random -lboost_regex -lboost_serialization -lboost_wserialization -lboost_coroutine -lboost_fiber -lboost_context -lboost_timer -lboost_thread -lboost_chrono -lboost_date_time -lboost_atomic -lboost_filesystem -lboost_system -lboost_type_erasure -lboost_log_setup -lboost_math_c99 -lboost_math_c99f -lboost_math_c99l -lboost_math_tr1 -lboost_math_tr1f -lboost_math_tr1l -lboost_nowide -lboost_stacktrace_noop -lboost_stacktrace_windbg -lboost_stacktrace_windbg_cached -lboost_unit_test_framework
CONAN_SYSTEMLIBS += -lbcrypt
CONAN_FRAMEWORKS += 
CONAN_FRAMEWORK_PATHS += 
CONAN_LIBDIRS += -L"C:/.conan/bdd45f/1/lib"
CONAN_BINDIRS += "C:/.conan/bdd45f/1/lib"
CONAN_RESDIRS += 
CONAN_BUILDDIRS += "C:/.conan/d97b85/1/" \
    "C:/.conan/bdd45f/1/" \
    "C:/Users/nduong/.conan/data/common/1.0.2/conan-solar/stable/package/5ab84d6acfe1f23c4fae0ab88f26e3a396351ac9/"
CONAN_DEFINES += "BOOST_ALL_DYN_LINK" \
    "BOOST_ALL_NO_LIB"
CONAN_QMAKE_CXXFLAGS += 
CONAN_QMAKE_CFLAGS += 
CONAN_QMAKE_LFLAGS += 
CONAN_QMAKE_LFLAGS += 

CONAN_INCLUDEPATH_EIGEN += "C:/.conan/d97b85/1/include" \
    "C:/.conan/d97b85/1/include/eigen3"
CONAN_LIBS_EIGEN += 
CONAN_SYSTEMLIBS_EIGEN += 
CONAN_FRAMEWORKS_EIGEN += 
CONAN_FRAMEWORK_PATHS_EIGEN += 
CONAN_LIBDIRS_EIGEN += 
CONAN_BINDIRS_EIGEN += 
CONAN_RESDIRS_EIGEN += 
CONAN_BUILDDIRS_EIGEN += "C:/.conan/d97b85/1/"
CONAN_DEFINES_EIGEN += 
CONAN_QMAKE_CXXFLAGS_EIGEN += 
CONAN_QMAKE_CFLAGS_EIGEN += 
CONAN_QMAKE_LFLAGS_EIGEN += 
CONAN_QMAKE_LFLAGS_EIGEN += 
CONAN_EIGEN_ROOT = "C:/.conan/d97b85/1"

CONAN_INCLUDEPATH_BOOST += "C:/.conan/bdd45f/1/include"
CONAN_LIBS_BOOST += -lboost_wave -lboost_container -lboost_contract -llibboost_exception -lboost_graph -lboost_iostreams -lboost_locale -lboost_log -lboost_program_options -lboost_random -lboost_regex -lboost_serialization -lboost_wserialization -lboost_coroutine -lboost_fiber -lboost_context -lboost_timer -lboost_thread -lboost_chrono -lboost_date_time -lboost_atomic -lboost_filesystem -lboost_system -lboost_type_erasure -lboost_log_setup -lboost_math_c99 -lboost_math_c99f -lboost_math_c99l -lboost_math_tr1 -lboost_math_tr1f -lboost_math_tr1l -lboost_nowide -lboost_stacktrace_noop -lboost_stacktrace_windbg -lboost_stacktrace_windbg_cached -lboost_unit_test_framework
CONAN_SYSTEMLIBS_BOOST += -lbcrypt
CONAN_FRAMEWORKS_BOOST += 
CONAN_FRAMEWORK_PATHS_BOOST += 
CONAN_LIBDIRS_BOOST += -L"C:/.conan/bdd45f/1/lib"
CONAN_BINDIRS_BOOST += "C:/.conan/bdd45f/1/lib"
CONAN_RESDIRS_BOOST += 
CONAN_BUILDDIRS_BOOST += "C:/.conan/bdd45f/1/"
CONAN_DEFINES_BOOST += "BOOST_ALL_DYN_LINK" \
    "BOOST_ALL_NO_LIB"
CONAN_QMAKE_CXXFLAGS_BOOST += 
CONAN_QMAKE_CFLAGS_BOOST += 
CONAN_QMAKE_LFLAGS_BOOST += 
CONAN_QMAKE_LFLAGS_BOOST += 
CONAN_BOOST_ROOT = "C:/.conan/bdd45f/1"

CONAN_INCLUDEPATH_COMMON += 
CONAN_LIBS_COMMON += 
CONAN_SYSTEMLIBS_COMMON += 
CONAN_FRAMEWORKS_COMMON += 
CONAN_FRAMEWORK_PATHS_COMMON += 
CONAN_LIBDIRS_COMMON += 
CONAN_BINDIRS_COMMON += 
CONAN_RESDIRS_COMMON += 
CONAN_BUILDDIRS_COMMON += "C:/Users/nduong/.conan/data/common/1.0.2/conan-solar/stable/package/5ab84d6acfe1f23c4fae0ab88f26e3a396351ac9/"
CONAN_DEFINES_COMMON += 
CONAN_QMAKE_CXXFLAGS_COMMON += 
CONAN_QMAKE_CFLAGS_COMMON += 
CONAN_QMAKE_LFLAGS_COMMON += 
CONAN_QMAKE_LFLAGS_COMMON += 
CONAN_COMMON_ROOT = "C:/Users/nduong/.conan/data/common/1.0.2/conan-solar/stable/package/5ab84d6acfe1f23c4fae0ab88f26e3a396351ac9"

CONFIG(conan_basic_setup) {
    INCLUDEPATH += $$CONAN_INCLUDEPATH
    LIBS += $$CONAN_LIBS
    LIBS += $$CONAN_LIBDIRS
    BINDIRS += $$CONAN_BINDIRS
    DEFINES += $$CONAN_DEFINES
    CONFIG(release, debug|release) {
        INCLUDEPATH += $$CONAN_INCLUDEPATH_RELEASE
        LIBS += $$CONAN_LIBS_RELEASE
        LIBS += $$CONAN_LIBDIRS_RELEASE
        BINDIRS += $$CONAN_BINDIRS_RELEASE
        DEFINES += $$CONAN_DEFINES_RELEASE
    } else {
        INCLUDEPATH += $$CONAN_INCLUDEPATH_DEBUG
        LIBS += $$CONAN_LIBS_DEBUG
        LIBS += $$CONAN_LIBDIRS_DEBUG
        BINDIRS += $$CONAN_BINDIRS_DEBUG
        DEFINES += $$CONAN_DEFINES_DEBUG
    }
    LIBS += $$CONAN_SYSTEMLIBS
    CONFIG(release, debug|release) {
        LIBS += $$CONAN_SYSTEMLIBS_RELEASE
    } else {
        LIBS += $$CONAN_SYSTEMLIBS_DEBUG
    }
    LIBS += $$CONAN_FRAMEWORKS
    LIBS += $$CONAN_FRAMEWORK_PATHS
    CONFIG(release, debug|release) {
        LIBS += $$CONAN_FRAMEWORKS_RELEASE
        LIBS += $$CONAN_FRAMEWORK_PATHS_RELEASE
    } else {
        LIBS += $$CONAN_FRAMEWORKS_DEBUG
        LIBS += $$CONAN_FRAMEWORK_PATHS_DEBUG
    }
    QMAKE_CXXFLAGS += $$CONAN_QMAKE_CXXFLAGS
    QMAKE_CFLAGS += $$CONAN_QMAKE_CFLAGS
    QMAKE_LFLAGS += $$CONAN_QMAKE_LFLAGS
    QMAKE_CXXFLAGS_DEBUG += $$CONAN_QMAKE_CXXFLAGS_DEBUG
    QMAKE_CFLAGS_DEBUG += $$CONAN_QMAKE_CFLAGS_DEBUG
    QMAKE_LFLAGS_DEBUG += $$CONAN_QMAKE_LFLAGS_DEBUG
    QMAKE_CXXFLAGS_RELEASE += $$CONAN_QMAKE_CXXFLAGS_RELEASE
    QMAKE_CFLAGS_RELEASE += $$CONAN_QMAKE_CFLAGS_RELEASE
    QMAKE_LFLAGS_RELEASE += $$CONAN_QMAKE_LFLAGS_RELEASE
}