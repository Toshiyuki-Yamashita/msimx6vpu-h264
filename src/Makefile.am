pluginsdir=$(PACKAGE_PLUGINS_DIR)

plugins_LTLIBRARIES=libmsimx6vpu_h264.la

libmsimx6vpu_h264_la_SOURCES=h264dec.c h264enc.c

libmsimx6vpu_h264_la_LIBADD=$(MEDIASTREAMER_LIBS)

libmsimx6vpu_h264_la_LDFLAGS=-module -no-undefined -lvpu

AM_CFLAGS=$(MEDIASTREAMER_CFLAGS)
