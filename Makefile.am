# Copyright 2023, Alistair Boyle, 3-clause BSD License

# the actual program to be installed at the end
bin_PROGRAMS = mcu-sim
mcu_sim_SOURCES = src/main.c src/argv.c src/armv7m.c
# header files that need to be distributed but not installed
noinst_HEADERS = src/*.h

AM_CFLAGS = $(CFLAGS) -I"$(top_srcdir)"/src

# some random files we want to distribute with the package tar
EXTRA_DIST = ChangeLog README.md NEWS LICENSE \
             tests/cmocka.h tests/cmocka_private.h \
             build-aux/git-version-gen \
             atlocal atconfig
DISTCLEANFILES = atlocal atconfig

CONFIGURE_DEPENDENCIES = $(top_srcdir)/.version
$(top_srcdir)/.version:
	GIT_DIR="$(top_srcdir)" git-version-gen $@
dist-hook: $(top_srcdir)/.version
	@cp $< $(distdir)/.tarball-version

check_PROGRAMS = tests/argv tests/armv7m tests/armv7m-test
tests_argv_SOURCES = tests/argv.c tests/cmocka.c
tests_armv7m_SOURCES = tests/armv7m.c tests/cmocka.c src/armv7m.c
TESTS = $(check_PROGRAMS)

tests/armv7m-test.bin: tests/*.c tests/*.h tests/*.ld tests/*.mk
	make -C tests -f armv7m.mk # $(@F)

tests/armv7m-test$(EXEEXT): tests/armv7m-test.bin
	echo -e "#! /bin/sh -e\nexec timeout 5s ./mcu-sim $<" > $@
	chmod +x $@

test: check

clean-local:
	-rm -f src/*.lo
	rm -f {src,tests}/*.gcda {src,tests}/*.gcno
	make -C tests -f armv7m.mk clean
	rm -f tests/armv7m-test$(EXEEXT)

maintainer-clean-local:
	-rm -f aclocal.m4 configure Makefile.in src/config.h.in*
	-rm -rf build-aux .deps


.PHONY: cov
cov.log: dist
	$(srcdir)/cov | tee cov.log
cov: cov.log
#	[ "x$(shell tail -4 $^ | grep '100.0%' | wc -l)" == "x3" ] || exit 1 # 100% test coverage please!

STYLE_FILES=src/*.c src/*.h tests/*.c tests/*.h
style: style.log
style.log: $(STYLE_FILES) astylerc
	astyle --suffix=none --options=./astylerc $(STYLE_FILES) | tee $@
check_style: style.log
	-grep Formatted style.log
	T_V[ "x$$(shell grep -c Formatted style.log)" == "x0" ] || exit 1
