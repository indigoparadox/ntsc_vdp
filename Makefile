
# vim: ft=make noexpandtab

SO_CFLAGS := -DRETROFLAT_OS_UNIX -DRETROFLAT_API_SDL1 `pkg-config --cflags sdl`
SO_LDFLAGS := `pkg-config --libs sdl`

all: retrovdp.so

retrovdp.so: retrovdp.c
	gcc -I../maug/src -fpic -shared -o $@ $< $(SO_CFLAGS) $(SO_LDFLAGS)

