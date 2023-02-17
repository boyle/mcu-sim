# Copyright 2023, Alistair Boyle, 3-clause BSD License
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])

AC_INIT([mcu-sim],
        @@GIT_VERSION@@,
        [alistair.js.boyle@gmail.com])

AC_COPYRIGHT([Copyright 2023, Alistair Boyle, 3-clause BSD License])

AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_AUX_DIR([build-aux])

AC_CONFIG_SRCDIR([src/main.c])
AC_CONFIG_HEADERS([src/config.h])
AC_USE_SYSTEM_EXTENSIONS
PKG_PROG_PKG_CONFIG

AC_CANONICAL_HOST
case $host_os in
   darwin* ) HOST_EXTRA_CFLAGS="-fgnu89-inline " ;;
   linux* ) ;;
   *mingw* ) ;;
   *)   AC_MSG_ERROR([unsupported host platform $host_os]) ;;
esac

# makefile
AM_INIT_AUTOMAKE([1.11 subdir-objects]) # need at least v1.11 of automake for SILENT_RULES
AM_SILENT_RULES([yes]) # enable kernel like cleaner build
AC_CONFIG_FILES([Makefile])

# Checks for programs.
AC_PROG_CC
AX_COMPILER_VENDOR
AX_COMPILER_VERSION
CFLAGS+=" -std=c17 ${HOST_EXTRA_CFLAGS}-Wall -Werror"
AM_PROG_CC_C_O

AC_C_INLINE
AC_TYPE_SIZE_T
AC_FUNC_MALLOC
AC_FUNC_REALLOC
AC_TYPE_UINT32_T
AC_FUNC_ERROR_AT_LINE
AC_FUNC_STRNLEN
AC_FUNC_STRTOD # argv.c
AC_HEADER_ASSERT
AC_CHECK_HEADERS_ONCE([malloc.h inttypes.h signal.h strings.h])
AC_CHECK_HEADERS([stdlib.h sys/time.h stddef.h limits.h],,AC_MSG_ERROR([missing header]))
AC_CHECK_FUNCS([malloc gettimeofday strndup memset strcasecmp strchr strdup strerror signal pow])
AC_CHECK_FUNCS([memmove strstr]) # cmocka.c
AC_CHECK_FUNCS([pow])
#AX_GNU_AUTOTEST

# Checks for libraries.
#AX_CHECK_ZLIB

# Setup the test suite
AC_CONFIG_TESTDIR([.],[.:tests])
AC_CONFIG_FILES([atlocal])

AC_MSG_CHECKING(for beer in the fridge)
  AC_MSG_RESULT(all systems go)

AC_MSG_RESULT([])
AC_MSG_RESULT([m4_text_box([Configuration Summary])])
AC_MSG_RESULT([  C Compiler: $CC ($build_os->$host_os/$ax_cv_c_compiler_vendor-$ax_cv_c_compiler_version)])
AC_MSG_RESULT([      CFLAGS: $CFLAGS])
AC_MSG_RESULT([     LDFLAGS: $LDFLAGS])
AC_MSG_RESULT([        LIBS: $LIBS])
AC_MSG_RESULT([])

AC_OUTPUT
GIT_VERSION=`cat ${srcdir}/.version | sed 's/.*= *//'`
sed -i "s/@@GIT_VERSION@@/${GIT_VERSION}/" config.status
